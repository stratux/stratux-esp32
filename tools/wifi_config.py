#!/usr/bin/env python3
"""Configure the ESP32's WiFi client mode and GDL90 targets over USB serial.

Drives the firmware's '$' command channel on the console UART0 (see
components/console_cmd/): a line like `$WIFI SET ssid="HangarNet" ...` is sent
and the board replies with a single `$OK ...` or `$ERR ...` line, which this
tool filters out of the interleaved firmware log stream.

Usage:
    wifi_config.py -p /dev/cu.usbserial-XXXX get
    wifi_config.py -p <port> set --enable --ssid HangarNet --pass s3cret
    wifi_config.py -p <port> set --dest 192.168.1.50,192.168.1.51
    wifi_config.py -p <port> set --disable
    wifi_config.py -p <port> reboot

`set` saves to NVS; WiFi changes need a reboot to apply (the tool reminds you;
--dest changes apply live). Exit codes: 0 = $OK, 1 = $ERR, 2 = timeout/serial
error.

Requires pyserial (`pip install pyserial`).
"""
import argparse
import sys
import time

try:
    import serial
except ImportError:
    sys.exit("error: pyserial not installed (pip install pyserial)")


def open_port(port, baud):
    """Open the console port WITHOUT resetting the board.

    The ESP32's onboard USB bridge wires DTR/RTS to GPIO0/EN; pyserial asserts
    DTR on open by default on some platforms, which resets the chip (or worse,
    drops it into the silent ROM download mode). Deassert both lines BEFORE
    opening so the board never sees a pulse (same issue pong_replay.py works
    around post-open; pre-open is stricter since we never want a reset here).
    """
    ser = serial.Serial()
    ser.port = port
    ser.baudrate = baud
    ser.timeout = 0.2
    ser.dtr = False
    ser.rts = False
    try:
        ser.open()
    except serial.SerialException as e:
        sys.exit(f"error: cannot open {port}: {e}")
    ser.dtr = False  # belt and braces: some drivers re-assert on open
    ser.rts = False
    ser.reset_input_buffer()
    return ser


def quote(value):
    """Double-quote a value for the command grammar (\\ and " escaped)."""
    return '"' + value.replace("\\", "\\\\").replace('"', '\\"') + '"'


def command(ser, line, timeout, verbose, attempts=3):
    """Send one '$' command and return the '$OK'/'$ERR' reply line.

    Sent in a retry loop: despite the pre-open DTR/RTS handling, some host
    drivers (macOS notably) still pulse the lines on open and reset the board,
    and a command sent during the ~2 s boot is silently discarded. A resend
    after a quiet period lands once the firmware is up.
    """
    for attempt in range(attempts):
        if verbose:
            note = " (retry)" if attempt else ""
            print(f"> {line}{note}", file=sys.stderr)
        ser.write((line + "\n").encode())
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            raw = ser.readline()
            if not raw:
                continue
            text = raw.decode("ascii", errors="replace").strip()
            if text.startswith("$OK") or text.startswith("$ERR"):
                return text
            if verbose and text:
                print(f"  {text}", file=sys.stderr)  # interleaved firmware log
    return None


def run(ser, lines, timeout, verbose):
    """Run commands in order, printing replies; fail fast. Returns exit code."""
    for line in lines:
        reply = command(ser, line, timeout, verbose)
        if reply is None:
            print(f"timeout waiting for reply to: {line}", file=sys.stderr)
            return 2
        print(reply)
        if reply.startswith("$ERR"):
            return 1
    return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("-p", "--port", required=True, help="console serial device (USB)")
    ap.add_argument("-b", "--baud", type=int, default=115200,
                    help="console baud (default 115200)")
    ap.add_argument("--timeout", type=float, default=5.0,
                    help="seconds to wait for each reply (default 5)")
    ap.add_argument("-v", "--verbose", action="store_true",
                    help="echo commands and interleaved firmware log lines")
    sub = ap.add_subparsers(dest="cmd", required=True)

    sub.add_parser("get", help="show WiFi client state and GDL90 targets")

    sp = sub.add_parser("set", help="update WiFi client settings / GDL90 targets")
    sp.add_argument("--ssid", help="network to join (1-32 chars)")
    sp.add_argument("--pass", dest="password", help="passphrase (8-64 chars, omit for open)")
    en = sp.add_mutually_exclusive_group()
    en.add_argument("--enable", action="store_true", help="turn client mode on")
    en.add_argument("--disable", action="store_true", help="turn client mode off")
    sp.add_argument("--dest", help="static GDL90 target IPs, comma-separated "
                                   "(max 4; empty string clears)")

    sub.add_parser("reboot", help="reboot the board (applies saved WiFi settings)")

    args = ap.parse_args()

    lines = []
    if args.cmd == "get":
        lines = ["$WIFI GET", "$DEST GET"]
    elif args.cmd == "reboot":
        lines = ["$REBOOT"]
    else:
        kv = []
        if args.enable:
            kv.append("sta_en=1")
        if args.disable:
            kv.append("sta_en=0")
        if args.ssid is not None:
            kv.append(f"ssid={quote(args.ssid)}")
        if args.password is not None:
            kv.append(f"pass={quote(args.password)}")
        if kv:
            lines.append("$WIFI SET " + " ".join(kv))
        if args.dest is not None:
            lines.append(f"$DEST SET dest={quote(args.dest)}")
        if not lines:
            ap.error("set: nothing to set (use --ssid/--pass/--enable/--disable/--dest)")

    ser = open_port(args.port, args.baud)
    try:
        rc = run(ser, lines, args.timeout, args.verbose)
    finally:
        ser.close()
    sys.exit(rc)


if __name__ == "__main__":
    main()
