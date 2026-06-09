"""Extract UTC time anchors from UAT uplink (FIS-B) frames and fit a capture's
line-number -> wall-clock mapping.

The legacy `ponglog-*.log` captures carry no timestamps. UAT uplink ('+') lines
are full FIS-B ground-uplink frames (432 bytes) whose info-frames embed a UTC
time field (hours/minutes, sometimes seconds and month/day). Individual product
times are noisy (forecasts and observations stamp future/past validity), but in
any short window the *current* time dominates by frequency — so a per-bin modal
time, regressed against line number, recovers the capture's real timeline.

Frame layout ported/verified against Stratux `uatparse/uatparse.go`
(DecodeUplink + decodeTimeFormat). See AGENTS.md "Protocol gotchas".
"""
from collections import Counter

UPLINK_FRAME_DATA_BYTES = 432
UPLINK_MAX_INFO_FRAMES = 100


def decode_uplink_times(hexstr):
    """Return list of (sec_of_day, month, day) for FIS-B info-frames carrying a
    time field in this uplink payload (hex string, '+'/';' already stripped).
    Returns [] if not a valid/long uplink or app data invalid.
    """
    try:
        raw = bytes.fromhex(hexstr)
    except ValueError:
        return []
    if len(raw) < UPLINK_FRAME_DATA_BYTES:
        # Short reads: Stratux right-pads to 432. Only worth it if there's a
        # plausible amount of app data; tiny fragments carry no FIS-B time.
        if len(raw) < 16:
            return []
        raw = raw + b"\x00" * (UPLINK_FRAME_DATA_BYTES - len(raw))

    if not (raw[6] & 0x20):          # app_data_valid
        return []
    app = raw[8:UPLINK_FRAME_DATA_BYTES]

    out = []
    pos = 0
    n = 0
    while n < UPLINK_MAX_INFO_FRAMES and pos + 2 <= len(app):
        d = app[pos:]
        flen = (d[0] << 1) | (d[1] >> 7)
        ftype = d[1] & 0x0f
        if flen == 0 or pos + flen > len(app):
            break
        rd = d[2:flen + 2]
        pos += 2 + flen
        n += 1
        if ftype != 0 or len(rd) < 3:   # only FIS-B APDU frames carry the time
            continue

        t_opt = ((rd[1] & 0x01) << 1) | (rd[2] >> 7)
        h = mi = se = None
        mo = da = None
        if t_opt == 0 and flen >= 4:                       # H, M
            h = (rd[2] & 0x7c) >> 2
            mi = ((rd[2] & 0x03) << 4) | (rd[3] >> 4)
        elif t_opt == 1 and flen >= 5:                     # H, M, S
            h = (rd[2] & 0x7c) >> 2
            mi = ((rd[2] & 0x03) << 4) | (rd[3] >> 4)
            se = ((rd[3] & 0x0f) << 2) | (rd[4] >> 6)
        elif t_opt == 2 and flen >= 5:                     # Mon, Day, H, M
            mo = (rd[2] & 0x78) >> 3
            da = ((rd[2] & 0x07) << 2) | (rd[3] >> 6)
            h = (rd[3] & 0x3e) >> 1
            mi = ((rd[3] & 0x01) << 5) | (rd[4] >> 3)
        elif t_opt == 3 and flen >= 6:                     # Mon, Day, H, M, S
            mo = (rd[2] & 0x78) >> 3
            da = ((rd[2] & 0x07) << 2) | (rd[3] >> 6)
            h = (rd[3] & 0x3e) >> 1
            mi = ((rd[3] & 0x01) << 5) | (rd[4] >> 3)
            se = ((rd[4] & 0x03) << 3) | (rd[5] >> 5)
        else:
            continue

        if h is None or h >= 24 or mi >= 60:
            continue
        out.append((h * 3600 + mi * 60 + (se or 0), mo, da))
    return out


def collect_anchors(lines):
    """From an iterable of raw Pong lines, return (anchors, dates) where
    anchors is [(line_idx, sec_of_day), ...] and dates is a Counter of
    (month, day) seen in dated FIS-B frames.
    """
    anchors = []
    dates = Counter()
    for idx, line in enumerate(lines):
        line = line.strip()
        if not line or line[0] != "+":
            continue
        hx = line[1:].split(";")[0]
        if len(hx) < 200:            # skip short non-FIS-B uplink fragments
            continue
        for sec, mo, da in decode_uplink_times(hx):
            anchors.append((idx, sec))
            if mo and da and 1 <= mo <= 12 and 1 <= da <= 31:
                dates[(mo, da)] += 1
    return anchors, dates


def _lsq(points):
    """Least-squares slope/intercept for [(x, y), ...]. Returns (slope, b)."""
    n = len(points)
    sx = sum(x for x, _ in points)
    sy = sum(y for _, y in points)
    sxx = sum(x * x for x, _ in points)
    sxy = sum(x * y for x, y in points)
    denom = n * sxx - sx * sx
    if denom == 0:
        return 0.0, sy / n
    slope = (n * sxy - sx * sy) / denom
    b = (sy - slope * sx) / n
    return slope, b


def fit_line_to_time(anchors, bin_size=4000, reject_sec=120.0):
    """Fit sec_of_day = slope*line + b from noisy anchors.

    Per line-bin, take the modal minute (the current time dominates each
    window). The catch: top-of-hour forecast products stamp a constant time
    (e.g. 20:00) that appears in *every* bin and wins the mode in some, so a
    plain least-squares seed collapses toward slope 0. Instead seed with
    deterministic all-pairs RANSAC over the bin-modal points — the monotonic
    current-time bins outnumber the flat forecast cluster as inliers — then
    refine by least-squares on that inlier set.

    Returns (slope_sec_per_line, b_sec, n_bins_used) or None if too little
    signal. Only positive (forward-in-time) slopes are accepted.
    """
    if len(anchors) < 4:
        return None
    bins = {}
    for idx, sec in anchors:
        bins.setdefault(idx // bin_size, []).append(sec // 60)   # minute resolution
    pts = []
    for b, mins in bins.items():
        if len(mins) < 3:                # ignore thin bins (mostly forecast noise)
            continue
        modal_min = Counter(mins).most_common(1)[0][0]
        pts.append((b * bin_size + bin_size / 2.0, modal_min * 60.0))
    if len(pts) < 3:
        return None

    # RANSAC seed: best line through any two points by inlier count.
    best = None
    for i in range(len(pts)):
        for j in range(i + 1, len(pts)):
            (x1, y1), (x2, y2) = pts[i], pts[j]
            if x2 == x1:
                continue
            s = (y2 - y1) / (x2 - x1)
            if s <= 0:                   # capture time only moves forward
                continue
            c = y1 - s * x1
            inliers = [(x, y) for x, y in pts if abs(y - (s * x + c)) <= reject_sec]
            score = (len(inliers), -sum(abs(y - (s * x + c)) for x, y in inliers))
            if best is None or score > best[0]:
                best = (score, inliers)
    if best is None or len(best[1]) < 3:
        return None

    slope, b = _lsq(best[1])
    if slope <= 0:
        return None
    return slope, b, len(best[1])
