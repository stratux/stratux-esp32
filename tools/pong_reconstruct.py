#!/usr/bin/env python3
"""Reconstruct a timestamped capture from a legacy un-timestamped Pong log.

The old `ponglog-*.log` files are raw Pong lines with no timestamps. This tool
synthesizes a UTC-millisecond timestamp per line so the result can be fed to
pong_replay.py. Timestamps come from a line-number -> wall-clock fit derived
from UTC time fields embedded in the UAT uplink (FIS-B) frames (see
fisb_time.py); lines are then spaced at the fitted constant rate. The absolute
date comes from dated FIS-B frames, the filename, or --date.

The per-line jitter of the original capture is unrecoverable, so timestamps are
evenly spaced at the inferred mean rate between the FIS-B wall-clock anchors —
a faithful estimate of pace and absolute time, not a claim of exact arrival.

For ponglog-07062025.log this infers ~48 lines/s over ~22.6 min starting
~2025-07-06 19:57:50 UTC.

Usage:
    pong_reconstruct.py ponglog-07062025.log -o ponglog-07062025.ponglog
    pong_reconstruct.py in.log -o out.ponglog --date 2025-07-06
    pong_reconstruct.py in.log -o out.ponglog --rate 48   # skip FIS-B, force rate
"""
import argparse
import calendar
import os
import re
import sys
import time

from fisb_time import collect_anchors, fit_line_to_time
from pong_capture_format import format_record


def date_from_filename(path):
    """Parse a YYYY or DDMMYYYY/MMDDYYYY date hint from 'ponglog-07062025.log'.
    Returns (year, month, day) or None. Ambiguous MM/DD vs DD/MM is resolved by
    the caller using FIS-B month/day when available.
    """
    m = re.search(r"(\d{2})(\d{2})(\d{4})", os.path.basename(path))
    if not m:
        return None
    a, b, year = int(m.group(1)), int(m.group(2)), int(m.group(3))
    return year, a, b   # tentatively (year, AA, BB); disambiguated later


def resolve_date(args, path, fisb_dates):
    if args.date:
        y, mo, da = (int(x) for x in args.date.split("-"))
        return y, mo, da
    fn = date_from_filename(path)
    # Prefer FIS-B's modal (month, day) for month/day; take year from filename.
    if fisb_dates:
        (mo, da), _ = fisb_dates.most_common(1)[0]
        year = fn[0] if fn else time.gmtime().tm_year
        return year, mo, da
    if fn:
        year, a, b = fn
        # Disambiguate AA/BB: if one is >12 it must be the day.
        if a > 12 >= b:
            return year, b, a
        return year, a, b   # assume MMDD
    sys.exit("error: could not determine date; pass --date YYYY-MM-DD")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("input", help="legacy un-timestamped Pong log")
    ap.add_argument("-o", "--out", required=True, help="output timestamped capture")
    ap.add_argument("--date", help="override capture date, YYYY-MM-DD (UTC)")
    ap.add_argument("--rate", type=float, default=None,
                    help="force lines/sec instead of FIS-B fit (start = midnight "
                         "or --start)")
    ap.add_argument("--start", help="override start time, HH:MM:SS (UTC)")
    args = ap.parse_args()

    with open(args.input, errors="replace") as fh:
        lines = [ln.rstrip("\r\n") for ln in fh]
    if not lines:
        sys.exit("error: empty input")
    n = len(lines)

    anchors, fisb_dates = collect_anchors(lines)
    year, mon, day = resolve_date(args, args.input, fisb_dates)
    midnight = calendar.timegm((year, mon, day, 0, 0, 0, 0, 0, 0))  # UTC epoch s

    if args.rate:
        slope = 1.0 / args.rate
        if args.start:
            h, m, s = (int(x) for x in args.start.split(":"))
            b = h * 3600 + m * 60 + s
        else:
            b = 0.0
        src = f"forced rate {args.rate} lines/s"
    else:
        fit = fit_line_to_time(anchors)
        if fit is None:
            sys.exit("error: insufficient FIS-B time anchors to fit; pass --rate")
        slope, b, nbins = fit
        if slope <= 0:
            sys.exit(f"error: nonsensical fit slope {slope}; pass --rate")
        if args.start:                                  # override start, keep rate
            h, m, s = (int(x) for x in args.start.split(":"))
            b = h * 3600 + m * 60 + s
        src = f"FIS-B fit over {len(anchors)} anchors / {nbins} bins"

    start_s = midnight + b
    end_s = midnight + b + slope * (n - 1)
    print(f"date {year:04d}-{mon:02d}-{day:02d} UTC | {src}", file=sys.stderr)
    print(f"rate {1.0 / slope:.2f} lines/s | "
          f"start {_hms(b)} end {_hms(b + slope * (n - 1))} | "
          f"duration {slope * (n - 1) / 60:.1f} min", file=sys.stderr)

    prev_ms = -1
    with open(args.out, "w") as out:
        out.write(f"# pong-capture v1 reconstructed_from={os.path.basename(args.input)}\n")
        out.write(f"# date={year:04d}-{mon:02d}-{day:02d} source={src!r} "
                  f"rate_lines_per_s={1.0 / slope:.3f}\n")
        for i, payload in enumerate(lines):
            if not payload:
                continue
            ms = int(round((start_s + slope * i) * 1000))
            if ms <= prev_ms:        # keep timestamps strictly nondecreasing
                ms = prev_ms + 1
            prev_ms = ms
            out.write(format_record(ms, payload) + "\n")

    print(f"wrote {n} lines -> {args.out}", file=sys.stderr)


def _hms(sec):
    s = int(round(sec)) % 86400
    return f"{s // 3600:02d}:{(s % 3600) // 60:02d}:{s % 60:02d}"


if __name__ == "__main__":
    main()
