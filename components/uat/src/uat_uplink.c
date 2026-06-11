// FIS-B uplink product tally — see uat_uplink.h. stratux-esp32 original; frame
// walk ported from Stratux uatparse/uatparse.go DecodeUplink/decodeInfoFrame
// (verified against tools/fisb_time.py, which mirrors the same layout).
//
// Portable: no ESP-IDF dependencies, so tools/replay_uplink.c can compile this
// file host-side against a capture. Concurrency on device is single-writer
// (pong task) with unlocked readers — same best-effort convention as g_status.

#include "uat_uplink.h"
#include "uat_decode.h"   // uat_hex_to_bytes()
#include <string.h>

#define UPLINK_FRAME_BYTES     432
#define UPLINK_MAX_INFO_FRAMES 100
#define UPLINK_MIN_BYTES        16   // tiny fragments carry no FIS-B payload

static uat_uplink_stats_t s_stats;

static void tally_product(uint16_t id, int64_t now_ms)
{
    for (int i = 0; i < UAT_UPLINK_PRODUCT_SLOTS; i++) {
        fisb_product_stat_t *p = &s_stats.products[i];
        if (p->count && p->product_id == id) {
            p->count++;
            p->last_ms = now_ms;
            return;
        }
        if (!p->count) {            // first free slot (table is append-only)
            p->product_id = id;
            p->count = 1;
            p->last_ms = now_ms;
            return;
        }
    }
    s_stats.overflow++;
}

void uat_uplink_tally(const pong_frame_t *f, int64_t now_ms)
{
    if (!f || f->hex_len == 0)
        return;

    // Overlong hex is corruption, not a frame — uat_hex_to_bytes() would
    // silently truncate it to the buffer and tally garbage as valid.
    if (f->hex_len > UPLINK_FRAME_BYTES * 2) {
        s_stats.bad_frames++;
        return;
    }

    // Right-pad short reads to a full frame (Stratux does the same); the
    // info-frame walk stops at the first zero-length header in the padding.
    uint8_t buf[UPLINK_FRAME_BYTES] = {0};
    int nbytes = uat_hex_to_bytes(f->hex, f->hex_len, buf, sizeof(buf));
    if (nbytes < UPLINK_MIN_BYTES) {
        s_stats.bad_frames++;
        return;
    }

    if (!(buf[6] & 0x20)) {          // app-data-valid bit
        s_stats.no_app_data++;
        return;
    }
    s_stats.frames++;

    const uint8_t *app = &buf[8];
    const int app_len = UPLINK_FRAME_BYTES - 8;

    int pos = 0;
    for (int n = 0; n < UPLINK_MAX_INFO_FRAMES && pos + 2 <= app_len; n++) {
        const uint8_t *d = &app[pos];
        int flen  = (d[0] << 1) | (d[1] >> 7);
        int ftype = d[1] & 0x0f;
        if (flen == 0 || pos + 2 + flen > app_len)
            break;
        const uint8_t *rd = &d[2];
        pos += 2 + flen;

        if (ftype != 0) {            // not a FIS-B APDU
            s_stats.tisb_frames++;
            continue;
        }
        if (flen < 2)
            continue;                // too short to carry a product ID

        uint16_t product_id = (uint16_t)(((rd[0] & 0x1f) << 6) | (rd[1] >> 2));
        s_stats.info_frames++;
        tally_product(product_id, now_ms);
    }
}

void uat_uplink_get_stats(uat_uplink_stats_t *out)
{
    if (out)
        memcpy(out, &s_stats, sizeof(*out));
}

// Stratux gen_gdl90.go product_name_map, collapsed onto switch ranges and
// extended with the post-2018 FIS-B expansion products (G-AIRMET, CWA, Icing,
// Cloud Tops, Turbulence, Lightning) that table predates — all present in real
// 2025 captures.
const char *fisb_product_name(uint16_t id)
{
    switch (id) {
    case 0:  case 20:           return "METAR";
    case 1:  case 21:           return "TAF";
    case 2:  case 22:           return "SIGMET";
    case 3:  case 23:           return "Conv SIGMET";
    case 4:  case 11: case 24:  return "AIRMET";
    case 5:  case 25:           return "PIREP";
    case 6:  case 26:           return "Severe Wx";
    case 7:  case 27:           return "Winds Aloft";
    case 8:                     return "NOTAM";
    case 9:                     return "D-ATIS";
    case 10:                    return "Terminal Wx";
    case 12:                    return "SIGMET";
    case 13:                    return "SUA";
    case 14:                    return "G-AIRMET";
    case 15:                    return "CWA";
    case 70: case 71:           return "Icing";
    case 84:                    return "Cloud Tops";
    case 90: case 91:           return "Turbulence";
    case 103:                   return "Lightning";
    case 51: case 52: case 53: case 54: case 55: case 56:
    case 57: case 58: case 59: case 60: case 61: case 62:
                                return "NEXRAD";
    case 63:                    return "NEXRAD Regional";
    case 64:                    return "NEXRAD CONUS";
    case 81: case 82: case 83:  return "Tops";
    case 101: case 102: case 151: return "Lightning";
    case 201: case 202:         return "Surface";
    case 254:                   return "G-AIRMET";
    case 351:                   return "Time";
    case 352: case 353:         return "Status";
    case 401:                   return "Imagery";
    case 402: case 405: case 411: case 413: return "Text";
    case 403:                   return "Vector Imagery";
    case 404: case 412:         return "Symbols";
    case 600:                   return "Custom/Test";
    default:
        if (id >= 2000 && id <= 2005) return "Custom/Test";
        return "Unknown";
    }
}
