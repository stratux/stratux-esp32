"""Shared definitions for the timestamped Pong capture format.

A capture file is UTF-8 text, one record per line:

    <epoch_ms> <raw Pong line>

where `<epoch_ms>` is an integer UTC time in **milliseconds since the Unix
epoch** (time.time() * 1000) and `<raw Pong line>` is the verbatim line the
Pong emitted, with its trailing CR/LF stripped (the Pong line itself never
contains a newline). The two are separated by a single ASCII space.

Lines beginning with '#' are comments (a header is written by the capture and
reconstruct tools) and are ignored by the replay tool. Blank lines are ignored.

This format is consumed by `pong_replay.py` (injects back over serial honoring
the inter-line timing) and produced by `pong_capture.py` (live, from a real
Pong) and `pong_reconstruct.py` (offline, synthesizing timestamps for the
legacy un-timestamped `ponglog-*.log` captures).
"""

DEFAULT_PORT = "/dev/cu.usbserial-5C8C0111211"
# The Pong serial link is 3,000,000 baud, 8N1 (Stratux main/pong.go, AGENTS.md).
DEFAULT_BAUD = 3_000_000


def parse_record(line):
    """Parse one capture-file line into (epoch_ms:int, payload:str).

    Returns None for comments, blanks, or malformed lines.
    """
    line = line.rstrip("\r\n")
    if not line or line[0] == "#":
        return None
    sp = line.find(" ")
    if sp <= 0:
        return None
    ts = line[:sp]
    if not ts.isdigit():
        return None
    return int(ts), line[sp + 1:]


def format_record(epoch_ms, payload):
    """Build one capture-file line from an epoch-ms int and a raw payload."""
    return f"{int(epoch_ms)} {payload}"
