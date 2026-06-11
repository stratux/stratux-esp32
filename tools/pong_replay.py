#!/usr/bin/env python3
"""Replay a Pong capture into a dev board while reading the board's output back.

The main bring-up / debugging harness: it injects a timestamped capture (see
pong_capture_format.py) into the ESP32's Pong UART over serial, honoring the
original inter-line timing scaled by --rate, AND concurrently reads the board's
serial output (firmware logs, decoded results) on a reader thread, printing
both streams interleaved with relative timestamps:

    [+   0.000] > *8DAA03A2...;34411        # > injected toward the board (TX)
    [+   0.021] < I (1234) traffic: upsert  # < read back from the board   (RX)

Injection (TX) and monitoring (RX) can share one full-duplex port, or use two:
typically inject into the Pong UART on one adapter and watch the ESP32 console
(UART0, 115200 — see AGENTS.md) on its USB port via --monitor-port.

    --rate 1.0   real time (default)        --rate 10   10x faster
    --rate 0.5   half speed

Usage:
    pong_replay.py cap.ponglog -p /dev/cu.usbserial-XXXX            # same port both ways
    pong_replay.py cap.ponglog -p <pong-uart> --monitor-port <console> --monitor-baud 115200
    pong_replay.py cap.ponglog --no-monitor --rate 5               # inject only
    pong_replay.py cap.ponglog --dry-run                           # print, no serial
    pong_replay.py cap.ponglog --heartbeat                         # phantom '.' @ 1 Hz

Requires pyserial (`pip install pyserial`).
"""
import argparse
import sys
import threading
import time

from pong_capture_format import DEFAULT_PORT, format_record, parse_record

_print_lock = threading.Lock()
_tx_lock = threading.Lock()   # serialize line writes (replay vs heartbeat thread)
_t0 = None   # monotonic start, set in main()


def _emit(direction, text):
    """Thread-safe stdout print with a relative timestamp and direction marker."""
    el = 0.0 if _t0 is None else time.monotonic() - _t0
    with _print_lock:
        sys.stdout.write(f"[+{el:8.3f}] {direction} {text}\n")
        sys.stdout.flush()


def load(path):
    recs = []
    with open(path) as fh:
        for line in fh:
            r = parse_record(line)
            if r is not None:
                recs.append(r)
    return recs


def main():
    global _t0
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("capture", help="timestamped capture file to replay")
    ap.add_argument("-p", "--port", default=DEFAULT_PORT, help="inject (TX) serial device")
    ap.add_argument("-b", "--baud", type=int, default=115200,
                    help="inject baud rate (default 115200 — the console-replay "
                         "dev path; the onboard USB bridge can't do the radio's "
                         "3 Mbaud, see AGENTS.md)")
    ap.add_argument("--rate", type=float, default=1.0,
                    help="replay speed multiplier (>1 faster, <1 slower)")
    ap.add_argument("--max-gap", type=float, default=None, metavar="SEC",
                    help="clamp any inter-line gap (post-scaling) to at most SEC")
    ap.add_argument("--loop", action="store_true", help="replay forever")
    ap.add_argument("--eol", default="crlf", choices=["crlf", "lf", "none"],
                    help="line terminator to send (default crlf)")
    ap.add_argument("--heartbeat", nargs="?", type=float, const=1.0, default=None,
                    metavar="SEC",
                    help="also inject a phantom Pong heartbeat ('.') line every "
                         "SEC seconds (default off; SEC defaults to 1.0). The "
                         "firmware marks the Pong link up only on heartbeat "
                         "lines, which raw captures lack")
    ap.add_argument("--echo", action="store_true", help="also print injected (TX) lines")
    # Monitoring (read the board's output back).
    ap.add_argument("--monitor-port", default=None,
                    help="separate device to read the board's output from "
                         "(default: read back from the inject port)")
    ap.add_argument("--monitor-baud", type=int, default=None,
                    help="baud for --monitor-port (default: same as --baud)")
    ap.add_argument("--no-monitor", action="store_true", help="do not read the board")
    ap.add_argument("--monitor-out", default=None,
                    help="also append the board's output to this file as "
                         "<epoch_ms> <line>")
    ap.add_argument("--no-reset", action="store_true",
                    help="don't pulse the board's auto-reset into run mode at start "
                         "(by default we reset so boot logs are captured)")
    ap.add_argument("--boot-wait", type=float, default=1.5, metavar="SEC",
                    help="after reset, wait this long for the firmware to boot "
                         "before injecting (default 1.5)")
    ap.add_argument("--dry-run", action="store_true",
                    help="print injected lines to stdout with timing, no serial")
    args = ap.parse_args()

    if args.rate <= 0:
        sys.exit("error: --rate must be > 0")
    if args.heartbeat is not None and args.heartbeat <= 0:
        sys.exit("error: --heartbeat interval must be > 0")

    recs = load(args.capture)
    if not recs:
        sys.exit(f"error: no records in {args.capture}")

    span = (recs[-1][0] - recs[0][0]) / 1000.0
    print(f"loaded {len(recs)} lines spanning {span:.1f}s "
          f"({len(recs) / span:.1f} lines/s); replay rate {args.rate}x",
          file=sys.stderr)

    term = {"crlf": b"\r\n", "lf": b"\n", "none": b""}[args.eol]

    if args.dry_run:
        _t0 = time.monotonic()
        stop = threading.Event()
        hb = _start_heartbeat(args, tx=None, term=term, stop=stop)
        try:
            _run(recs, args, tx=None, term=term)
        except KeyboardInterrupt:
            print("\nstopped", file=sys.stderr)
        finally:
            stop.set()
            if hb is not None:
                hb.join(timeout=1.0)
        return

    try:
        import serial
    except ImportError:
        sys.exit("error: pyserial not installed — run `pip install pyserial`")

    # Open inject (TX) port.
    try:
        tx = serial.Serial(args.port, args.baud, timeout=1)
    except serial.SerialException as e:
        sys.exit(f"error: cannot open inject port {args.port}: {e}")

    # Resolve monitor (RX) port: separate device, the same port, or disabled.
    mon = None
    own_mon = False
    if not args.no_monitor:
        if args.monitor_port and args.monitor_port != args.port:
            mbaud = args.monitor_baud or args.baud
            try:
                mon = serial.Serial(args.monitor_port, mbaud, timeout=0.2)
                own_mon = True
            except serial.SerialException as e:
                tx.close()
                sys.exit(f"error: cannot open monitor port {args.monitor_port}: {e}")
            print(f"monitoring <- {args.monitor_port} @ {mbaud}", file=sys.stderr)
        else:
            mon = tx            # full-duplex on the inject port
            mon.timeout = 0.2
            print(f"monitoring <- {args.port} (same port, full-duplex)", file=sys.stderr)

    print(f"replaying -> {args.port} @ {args.baud} (Ctrl-C to stop)", file=sys.stderr)

    # The ESP32's onboard USB bridge wires DTR/RTS to GPIO0/EN. Opening a port
    # with DTR asserted (the pyserial default on some platforms) holds GPIO0 low
    # so a reset drops the chip into the silent ROM download mode — which looks
    # like "no console output". Deassert both lines on every port we hold so the
    # board always runs the app. The console port is the one wired to the reset
    # circuit (the separate monitor port, else the inject/same port).
    console = mon if own_mon else tx
    _quiesce(tx)
    if own_mon:
        _quiesce(mon)

    stop = threading.Event()
    reader = None
    mon_fh = open(args.monitor_out, "a", buffering=1) if args.monitor_out else None
    if mon is not None:
        reader = threading.Thread(target=_reader_loop, args=(mon, stop, mon_fh),
                                  daemon=True)
        reader.start()

    # Reset into run mode so we capture a fresh boot log, then let it come up.
    if not args.no_reset:
        _reset_to_run(console)
        if args.boot_wait > 0:
            time.sleep(args.boot_wait)

    _t0 = time.monotonic()
    hb = _start_heartbeat(args, tx=tx, term=term, stop=stop)
    try:
        _run(recs, args, tx=tx, term=term)
    except KeyboardInterrupt:
        print("\nstopped", file=sys.stderr)
    finally:
        stop.set()
        if hb is not None:
            hb.join(timeout=1.0)
        if reader is not None:
            reader.join(timeout=1.0)
        if own_mon and mon is not None:
            mon.close()
        tx.close()
        if mon_fh is not None:
            mon_fh.close()


def _quiesce(ser):
    """Deassert DTR and RTS so the ESP32 auto-reset circuit leaves GPIO0 high
    (run mode), never the silent ROM download mode. No-op if unsupported."""
    try:
        ser.dtr = False
        ser.rts = False
    except Exception:
        pass


def _reset_to_run(ser):
    """Classic ESP32 auto-reset into the application (not the bootloader):
    GPIO0 high (DTR deasserted) while EN is pulsed low->high (RTS)."""
    try:
        ser.dtr = False          # GPIO0 high -> normal boot
        ser.rts = True           # EN low  -> hold in reset
        time.sleep(0.1)
        ser.reset_input_buffer()
        ser.rts = False          # EN high -> release -> run app
    except Exception:
        pass


def _send_line(tx, payload, term, echo):
    """Write one full line under the TX lock (tx=None means dry-run/stdout)."""
    if tx is not None:
        with _tx_lock:
            tx.write(payload.encode("ascii", "replace") + term)
        if echo:
            _emit(">", payload)
    else:
        _emit(">", payload)


def _start_heartbeat(args, tx, term, stop):
    """If --heartbeat is set, start a thread injecting phantom '.' lines (the
    Pong heartbeat the firmware keys pong_connected on) every interval until
    `stop` is set. Returns the thread, or None when the flag is off."""
    if args.heartbeat is None:
        return None
    print(f"injecting phantom heartbeat ('.') every {args.heartbeat:g}s",
          file=sys.stderr)

    def loop():
        while not stop.wait(args.heartbeat):
            _send_line(tx, ".", term, args.echo)

    t = threading.Thread(target=loop, daemon=True)
    t.start()
    return t


def _run(recs, args, tx, term):
    """Inject all records (optionally looping). tx=None means dry-run/stdout."""
    while True:
        _replay_once(recs, args, tx, term)
        if not args.loop:
            break


def _replay_once(recs, args, tx, term):
    # Anchor wall clock to the pass start; schedule each line at its cumulative
    # offset so per-line sleep rounding doesn't accumulate into drift.
    wall0 = time.monotonic()
    offs = _offsets(recs, args)
    for i, (ts, payload) in enumerate(recs):
        target = wall0 + offs[i]
        now = time.monotonic()
        if target > now:
            time.sleep(target - now)
        _send_line(tx, payload, term, args.echo)


def _reader_loop(mon, stop, mon_fh):
    """Read the board's serial output, splitting into lines on CR/LF, printing
    each with a '<' marker. Runs until `stop` is set. Tolerant of partial reads
    and non-ASCII bytes."""
    buf = bytearray()
    while not stop.is_set():
        try:
            chunk = mon.read(4096)
        except Exception as e:                  # port yanked, etc.
            _emit("<", f"[monitor read error: {e}]")
            return
        if not chunk:
            continue
        buf.extend(chunk)
        while True:
            i = _find_eol(buf)
            if i < 0:
                break
            raw = bytes(buf[:i])
            del buf[:i + 1]
            if not raw:
                continue
            line = raw.decode("ascii", "replace").rstrip("\r")
            if not line:
                continue
            _emit("<", line)
            if mon_fh is not None:
                mon_fh.write(format_record(int(time.time() * 1000), line) + "\n")


def _find_eol(buf):
    n = buf.find(b"\n")
    r = buf.find(b"\r")
    if n < 0:
        return r
    if r < 0:
        return n
    return min(n, r)


def _offsets(recs, args):
    """Cumulative scheduled offsets (s) from pass start, one per record.

    Each original inter-line gap is scaled by 1/rate and individually clamped to
    --max-gap (if set), then accumulated so the schedule can't drift.
    """
    offs = [0.0]
    for k in range(1, len(recs)):
        g = (recs[k][0] - recs[k - 1][0]) / 1000.0 / args.rate
        if args.max_gap is not None and g > args.max_gap:
            g = args.max_gap
        offs.append(offs[-1] + g)
    return offs


if __name__ == "__main__":
    main()
