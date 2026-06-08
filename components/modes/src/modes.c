#include "modes.h"
#include "esp_log.h"
#include "stratux_status.h"
#include "traffic.h"

static const char *TAG = "modes";

void modes_decode_frame(const pong_frame_t *f)
{
    if (!f) return;

    // TODO(M1): the real decoder. Outline (port from dump1090 mode_s.c):
    //   1. hex -> bytes; check DF (bits 1-5). Handle DF17 (ADS-B) and DF18
    //      (CF -> ADS-R / TIS-B). Other DFs: extract altitude/ident only.
    //   2. CRC/parity (DF17/18 use the 24-bit parity = ICAO for DF17).
    //   3. Type code (bits 33-37): TC 1-4 ident/callsign, TC 9-18/20-22 airborne
    //      position (CPR even/odd, expiry), TC 5-8 surface position, TC 19
    //      velocity (gs + vrate, or airspeed). Fill NIC/NACp.
    //   4. Stage CPR even/odd per ICAO; on a valid pair, set position_valid.
    //   5. Build a traffic_info_t and call traffic_upsert(&t).

    g_status.es_msgs++;
    ESP_LOGV(TAG, "1090ES frame (decode TODO) ss=0x%04x", f->ss);
}
