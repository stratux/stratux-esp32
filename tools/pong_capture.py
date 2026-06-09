#!/usr/bin/env python3
"""Read live frames from a Pong over serial and write a timestamped capture.

Each received line is written as

    <epoch_ms> <raw Pong line>

(see pong_capture_format.py). Timestamps are UTC milliseconds taken the instant
the line's terminating newline is read off the wire.

Usage:
    pong_capture.py -o capture.ponglog
    pong_capture.py -p /dev/cu.usbserial-XXXX -b 3000000 -o out.ponglog --echo

The Pong streams newline-delimited ASCII at 3 Mbaud; the first char classifies
the line ('*' 1090ES, '+'/'-' UAT, '.' heartbeat, "'" status, else log). We do
not interpret it here — we just timestamp and store verbatim. Stop with Ctrl-C.

Requires pyserial (`pip install pyserial`).
"""
import argparse
import sys
import time

from pong_capture_format import DEFAULT_BAUD, DEFAULT_PORT, format_record


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("-p", "--port", default=DEFAULT_PORT, help="serial device")
    ap.add_argument("-b", "--baud", type=int, default=DEFAULT_BAUD, help="baud rate")
    ap.add_argument("-o", "--out", required=True, help="output capture file")
    ap.add_argument("--echo", action="store_true", help="also echo lines to stdout")
    args = ap.parse_args()

    try:
        import serial
    except ImportError:
        sys.exit("error: pyserial not installed — run `pip install pyserial`")

    try:
        ser = serial.Serial(args.port, args.baud, timeout=1)
    except serial.SerialException as e:
        sys.exit(f"error: cannot open {args.port}: {e}")

    n = 0
    started = time.time()
    # Accumulate bytes and split on CR/LF, matching the firmware's line framing.
    buf = bytearray()
    with open(args.out, "w", buffering=1) as fh:
        fh.write(f"# pong-capture v1 port={args.port} baud={args.baud}\n")
        fh.write(f"# created_epoch_ms={int(started * 1000)} "
                 f"created_utc={time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime(started))}\n")
        print(f"capturing from {args.port} @ {args.baud} -> {args.out} (Ctrl-C to stop)",
              file=sys.stderr)
        try:
            while True:
                chunk = ser.read(4096)
                if not chunk:
                    continue
                now_ms = int(time.time() * 1000)
                buf.extend(chunk)
                # Emit every complete line; keep the trailing partial in buf.
                while True:
                    i = _find_eol(buf)
                    if i < 0:
                        break
                    raw = bytes(buf[:i])
                    del buf[:i + 1]
                    if not raw:
                        continue
                    payload = raw.decode("ascii", "replace").rstrip("\r")
                    if not payload:
                        continue
                    fh.write(format_record(now_ms, payload) + "\n")
                    n += 1
                    if args.echo:
                        print(payload)
        except KeyboardInterrupt:
            pass
        finally:
            ser.close()

    dur = time.time() - started
    print(f"\ncaptured {n} lines in {dur:.1f}s ({n / dur:.1f} lines/s) -> {args.out}",
          file=sys.stderr)


def _find_eol(buf):
    """Index of the first \\n or \\r in buf, or -1."""
    n = buf.find(b"\n")
    r = buf.find(b"\r")
    if n < 0:
        return r
    if r < 0:
        return n
    return min(n, r)


if __name__ == "__main__":
    main()
