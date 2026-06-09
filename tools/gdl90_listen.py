#!/usr/bin/env python3
"""Listen for GDL90 datagrams on UDP :4000 and validate/decode them.

Use this to confirm the firmware's M0 heartbeat (and later traffic) emission:
join the device's "stratux" SoftAP (so its DHCP server leases you an address and
unicasts GDL90 to you — it never broadcasts), then run this. It deframes each
datagram, verifies the GDL90-variant CRC (NOT XMODEM — poly 0x1021, init 0,
data XORed into the low byte after 8 shifts; "123456789" -> 0xBEEF), decodes the
message id, and prints heartbeat fields plus the observed rate.

    gdl90_listen.py                 # bind 0.0.0.0:4000, run until Ctrl-C
    gdl90_listen.py --port 4000 --seconds 10 --raw

Pure stdlib; no pyserial needed.
"""
import argparse
import socket
import sys
import time

MSG_NAMES = {
    0x00: "Heartbeat", 0x07: "Uplink", 0x0A: "Ownship", 0x0B: "OwnshipGeoAlt",
    0x14: "Traffic", 0x65: "FF-ID/AHRS", 0x4C: "Levil-AHRS", 0xCC: "Stratux-HB",
}


def gdl90_crc16(data):
    """GDL90 ICD 2.2.4 CRC — shift 8 times then XOR the data byte into the low
    byte (matches the firmware's gdl90_crc16). Self-test: '123456789' -> 0xBEEF."""
    crc = 0
    for byte in data:
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if (crc & 0x8000) else (crc << 1) & 0xFFFF
        crc ^= byte
    return crc & 0xFFFF


def deframe(datagram):
    """Yield raw (unescaped) message bodies WITHOUT the trailing 2 CRC bytes,
    plus a crc_ok flag, for each 0x7E-delimited frame in a datagram."""
    # Split into frames on 0x7E flags, dropping empties.
    frames = []
    cur = bytearray()
    in_frame = False
    for b in datagram:
        if b == 0x7E:
            if in_frame and cur:
                frames.append(bytes(cur))
                cur = bytearray()
            in_frame = True
            continue
        if in_frame:
            cur.append(b)
    for f in frames:
        # Unescape: 0x7D 0xYY -> (0xYY ^ 0x20).
        out = bytearray()
        i = 0
        ok = True
        while i < len(f):
            if f[i] == 0x7D:
                if i + 1 >= len(f):
                    ok = False
                    break
                out.append(f[i + 1] ^ 0x20)
                i += 2
            else:
                out.append(f[i])
                i += 1
        if not ok or len(out) < 3:
            yield None, False
            continue
        body, crc_rx = bytes(out[:-2]), out[-2] | (out[-1] << 8)
        yield body, (gdl90_crc16(body) == crc_rx)


def decode_heartbeat(body):
    if len(body) < 7:
        return "short heartbeat"
    sb1, sb2 = body[1], body[2]
    ts = body[3] | (body[4] << 8) | (((sb2 >> 7) & 1) << 16)
    return (f"SB1=0x{sb1:02x}[UAT_init={sb1&1} addr_tb={(sb1>>4)&1} "
            f"gps_valid={(sb1>>7)&1} maint={(sb1>>6)&1}] "
            f"SB2=0x{sb2:02x}[UTC_OK={sb2&1}] ts={ts}s "
            f"counts={body[5]:02x}{body[6]:02x}")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--bind", default="0.0.0.0", help="bind address")
    ap.add_argument("--port", type=int, default=4000, help="UDP port (default 4000)")
    ap.add_argument("--seconds", type=float, default=None,
                    help="stop after N seconds (default: run until Ctrl-C)")
    ap.add_argument("--raw", action="store_true", help="also print raw hex of each frame")
    args = ap.parse_args()

    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        s.bind((args.bind, args.port))
    except OSError as e:
        sys.exit(f"error: cannot bind {args.bind}:{args.port}: {e}")
    s.settimeout(0.5)
    print(f"listening on {args.bind}:{args.port} (Ctrl-C to stop) — "
          f"join the 'stratux' AP so the device unicasts to you", file=sys.stderr)

    counts = {}
    bad_crc = 0
    first = last_hb = None
    hb_n = 0
    t0 = time.monotonic()
    try:
        while True:
            if args.seconds is not None and time.monotonic() - t0 >= args.seconds:
                break
            try:
                data, addr = s.recvfrom(2048)
            except socket.timeout:
                continue
            if first is None:
                first = time.monotonic()
                print(f"first datagram from {addr[0]}", file=sys.stderr)
            for body, ok in deframe(data):
                if body is None:
                    bad_crc += 1
                    continue
                if not ok:
                    bad_crc += 1
                    print(f"  CRC FAIL  {body.hex()}")
                    continue
                mid = body[0]
                counts[mid] = counts.get(mid, 0) + 1
                name = MSG_NAMES.get(mid, f"0x{mid:02x}")
                if args.raw:
                    print(f"  {name}: {body.hex()}")
                if mid == 0x00:
                    hb_n += 1
                    now = time.monotonic()
                    dt = f" (+{now - last_hb:.2f}s)" if last_hb else ""
                    last_hb = now
                    print(f"[{name} #{hb_n}{dt}] {decode_heartbeat(body)}")
    except KeyboardInterrupt:
        pass

    dur = (time.monotonic() - (first or t0))
    print("\n--- summary ---", file=sys.stderr)
    if not counts and not bad_crc:
        print("no GDL90 received — is the host joined to the 'stratux' AP, and is "
              "an EFB/this host leased? (device unicasts per DHCP lease)",
              file=sys.stderr)
    for mid, c in sorted(counts.items()):
        rate = c / dur if dur > 0 else 0
        print(f"{MSG_NAMES.get(mid, hex(mid)):14s} {c:6d}  ({rate:.2f}/s)", file=sys.stderr)
    if bad_crc:
        print(f"CRC/deframe failures: {bad_crc}", file=sys.stderr)


if __name__ == "__main__":
    main()
