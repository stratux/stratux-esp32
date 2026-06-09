#include "uat.h"
#include "esp_log.h"
#include "stratux_status.h"
#include "traffic.h"

static const char *TAG = "uat";

void uat_decode_frame(const pong_frame_t *f)
{
    if (!f) return;

    // TODO(M1): downlink decode. Outline (port from dump978 + uatparse):
    //   - hex -> bytes; identify SHORT (18 B) vs LONG (34 B) ADS-B message.
    //   - extract address, lat/lng, altitude, velocity, callsign, NIC/NACp.
    //   - build traffic_info_t (addr_type from the DO-282 address qualifier)
    //     and traffic_upsert().
    // TODO(M5): uplink frames ('+') -> FIS-B products (wxstore) or 0x07 relay.

    g_status.uat_msgs++;
    ESP_LOGV(TAG, "UAT frame kind=%d (decode TODO) rs=%u ss=0x%04x",
             f->kind, f->rs, f->ss);
}
