#include "gdl90_out.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "net.h"
#include "traffic.h"
#include "stratux_status.h"

static const char *TAG = "gdl90";

// Latched by gdl90_out_init(): true only if the CRC self-test produced 0xBEEF.
// gdl90_emit_task() refuses to run if this is false — a mis-built CRC would make
// every frame CRC-fail at the EFB, so silence is correct.
static bool s_crc_ok = false;

#define GDL90_FLAG  0x7E
#define GDL90_ESC   0x7D

// CRC-16 per GDL90 ICD §2.2.4 (poly 0x1021, init 0x0000) — see header banner.
// Tableless per-bit form (equivalent to the ICD table form); plenty fast at
// GDL90 rates on a 240 MHz core. Verbatim from connext aera660_gdl90_uart.c.
uint16_t gdl90_crc16(const uint8_t *data, size_t len)
{
    uint16_t crc = 0;
    for (size_t i = 0; i < len; i++) {
        for (int b = 0; b < 8; b++) {
            crc = (crc & 0x8000)
                  ? (uint16_t)((crc << 1) ^ 0x1021)
                  : (uint16_t)(crc << 1);
        }
        crc ^= (uint16_t)data[i];
    }
    return crc;
}

int gdl90_frame(const uint8_t *payload, size_t len, uint8_t *out, size_t out_cap)
{
    if (!payload || !out) return -1;
    uint16_t crc = gdl90_crc16(payload, len);
    const uint8_t crc_lo = (uint8_t)(crc & 0xFF);
    const uint8_t crc_hi = (uint8_t)(crc >> 8);

    size_t n = 0;
    #define PUT(b)  do { if (n >= out_cap) return -1; out[n++] = (uint8_t)(b); } while (0)
    PUT(GDL90_FLAG);
    // Escape every payload + CRC byte (CRC appended little-endian, low first).
    for (size_t i = 0; i < len + 2; i++) {
        uint8_t b = (i < len) ? payload[i] : (i == len ? crc_lo : crc_hi);
        if (b == GDL90_FLAG || b == GDL90_ESC) {
            PUT(GDL90_ESC);
            PUT(b ^ 0x20);
        } else {
            PUT(b);
        }
    }
    PUT(GDL90_FLAG);
    #undef PUT
    return (int)n;
}

void gdl90_out_init(void)
{
    // GDL90-variant self-test: "123456789" -> 0xBEEF (XMODEM would be 0x31C3).
    uint16_t v = gdl90_crc16((const uint8_t *)"123456789", 9);
    if (v != 0xBEEF) {
        s_crc_ok = false;
        ESP_LOGE(TAG, "CRC self-test FAILED: got 0x%04x, expected 0xBEEF — "
                      "GDL90 emit DISABLED (every frame would CRC-fail)", v);
        return;
    }
    s_crc_ok = true;
    ESP_LOGI(TAG, "CRC self-test ok: 0xBEEF (GDL90 ICD §2.2.4 variant)");
}

// Build the 7-byte GDL90 heartbeat (0x00). Byte layout per Stratux
// gen_gdl90.go makeHeartbeat (the reference sender EFBs trust).
static size_t build_heartbeat(uint8_t *p)
{
    memset(p, 0, 7);
    p[0] = GDL90_MSG_HEARTBEAT;

    // Status Byte 1: bit0 "UAT Initialized" + bit4 "Addr talkback" — both always
    // set by Stratux. bit7 (GPS pos valid) and bit6 (maintenance req'd) get
    // added once GPS / error wiring exists (M3+).
    p[1] = 0x01 | 0x10;

    // Status Byte 2 + 17-bit "seconds since 0000Z" timestamp: bit16 -> SB2 bit7,
    // low 16 bits -> p[3..4] little-endian; SB2 bit0 = "UTC OK".
    // AGENTS.md M0: with no clock, keep UTC OK CLEAR and the timestamp ZERO
    // rather than lying about time. Ready for the M3 time source.
    if (g_status.utc_ok) {
        uint32_t s = g_status.secs_since_midnight;
        p[2] = (uint8_t)(((s >> 16) << 7) | 0x01);
        p[3] = (uint8_t)(s & 0xFF);
        p[4] = (uint8_t)((s >> 8) & 0xFF);
    }
    // p[5..6]: messages received in the previous second (ICD §3.1.2): byte 5
    // bits 7-3 = uplink count (sat. 31), bits 1-0 + byte 6 = basic+long count
    // (sat. 1023). Stratux never implemented these (gen_gdl90.go TODO) so EFBs
    // tolerate zeros, but the real counters exist here — report them.
    {
        static uint32_t last_uplink, last_basic;
        uint32_t uplink = g_status.uat_uplink_msgs;
        uint32_t basic  = g_status.es_msgs + g_status.uat_msgs;
        uint32_t du = uplink - last_uplink;
        uint32_t db = basic - last_basic;
        last_uplink = uplink;
        last_basic = basic;
        if (du > 31) du = 31;
        if (db > 1023) db = 1023;
        p[5] = (uint8_t)((du << 3) | ((db >> 8) & 0x03));
        p[6] = (uint8_t)(db & 0xFF);
    }
    return 7;
}

// Build the 2-byte Stratux custom heartbeat (0xCC). Layout per Stratux
// gen_gdl90.go makeStratuxHeartbeat: bit1 = GPS valid (M3), bit0 = AHRS valid
// (M4) — both clear now; protocol version (1) sits in bits 2+.
static size_t build_stratux_heartbeat(uint8_t *p)
{
    p[0] = GDL90_MSG_STRATUX_HB;
    p[1] = (uint8_t)(1 << 2);   // protocol version 1; GPS/AHRS bits clear
    return 2;
}

// GDL90 lat/lon are 24-bit signed semicircles: degrees / (180 / 2^23).
#define GDL90_LATLNG_RES (180.0 / 8388608.0)

static void put_latlng(uint8_t *p, double deg)
{
    int32_t wk = (int32_t)(deg / GDL90_LATLNG_RES);
    p[0] = (uint8_t)((wk >> 16) & 0xFF);
    p[1] = (uint8_t)((wk >> 8) & 0xFF);
    p[2] = (uint8_t)(wk & 0xFF);
}

// traffic_addr_type_t is semantic, not wire format — map it to the GDL90 ICD
// address-type value explicitly so reordering/extending the enum can't change
// what goes on the wire. (GDL90's table descends from the UAT MASPS, so the
// DO-282 address qualifiers map 1:1; the UAT decoder classifies AQ per frame.)
static uint8_t gdl90_addr_type(traffic_addr_type_t at)
{
    switch (at) {
    case ADDR_TYPE_ADSB_ICAO:    return 0;  // ADS-B with ICAO address
    case ADDR_TYPE_ADSB_OTHER:   return 1;  // ADS-B with self-assigned address
    case ADDR_TYPE_TISB_ICAO:    return 2;  // TIS-B with ICAO address
    case ADDR_TYPE_TISB_OTHER:   return 3;  // TIS-B with track file ID
    case ADDR_TYPE_ADSB_VEHICLE: return 4;  // surface vehicle
    case ADDR_TYPE_FIXED_BEACON: return 5;  // ground station beacon
    }
    return 0;
}

// Build the 28-byte GDL90 Traffic Report (0x14). Byte layout ported verbatim
// from Stratux gen_gdl90.go makeTrafficReportMsg (the reference EFBs trust).
// Caller must only pass entries with a valid position.
static size_t build_traffic_report(uint8_t *p, const traffic_info_t *t)
{
    memset(p, 0, 28);
    p[0] = GDL90_MSG_TRAFFIC;

    // msg[1]: address type (low nibble). The alert bit (0x10) needs ownship
    // proximity logic — deferred to M3, left clear.
    p[1] = gdl90_addr_type(t->addr_type);

    // ICAO address.
    p[2] = (uint8_t)((t->icao_addr >> 16) & 0xFF);
    p[3] = (uint8_t)((t->icao_addr >> 8) & 0xFF);
    p[4] = (uint8_t)(t->icao_addr & 0xFF);

    // Latitude / longitude.
    put_latlng(&p[5], t->lat);
    put_latlng(&p[8], t->lng);

    // Altitude: 25 ft resolution, 1000 ft offset; 0xFFF = invalid/unavailable.
    int16_t encodedAlt;
    if (!t->alt_valid || t->alt_ft < -1000 || t->alt_ft > 101350)
        encodedAlt = 0x0FFF;
    else
        encodedAlt = (int16_t)((t->alt_ft / 25) + 40);
    p[11] = (uint8_t)((encodedAlt & 0x0FF0) >> 4);
    p[12] = (uint8_t)((encodedAlt & 0x000F) << 4);

    // "m" nibble: track validity, extrapolated, airborne.
    if (t->speed_valid)   p[12] |= 0x01;   // tt is true track
    if (t->extrapolated)  p[12] |= 0x04;   // report is extrapolated
    if (!t->on_ground)    p[12] |= 0x08;   // airborne

    // NIC / NACp. NACp arrives only in TC31 operational-status frames; when
    // none has been seen, seed it from NIC (Stratux does the same) instead of
    // reporting "unknown accuracy" for an aircraft with a good position.
    uint8_t nic  = t->nic_valid ? t->nic : 0;
    uint8_t nacp = t->nacp_valid ? t->nacp : nic;
    p[13] = (uint8_t)(((nic << 4) & 0xF0) | (nacp & 0x0F));

    // Horizontal velocity (12 bits): 0xFFF = no data.
    uint16_t spd = t->speed_valid ? (t->speed_kt > 0xFFE ? 0xFFE : t->speed_kt) : 0x0FFF;
    p[14] = (uint8_t)((spd & 0x0FF0) >> 4);
    p[15] = (uint8_t)((spd & 0x000F) << 4);

    // Vertical velocity (12-bit signed, 64 fpm units): 0x800 = no data.
    uint16_t vv;
    if (t->vvel_valid) {
        int v = t->vvel_fpm / 64;
        vv = (uint16_t)(v & 0x0FFF);
    } else {
        vv = 0x0800;
    }
    p[15] |= (uint8_t)((vv & 0x0F00) >> 8);
    p[16] = (uint8_t)(vv & 0x00FF);

    // Track / heading (8-bit, 360/256 deg resolution).
    p[17] = (uint8_t)(((int)t->track_deg * 256) / 360);

    // Emitter category.
    p[18] = t->cat_valid ? t->emitter_cat : 0;

    // Call sign / tail (msg[19..26]), sanitized to the GDL90 charset.
    size_t taillen = t->tail_valid ? strlen(t->tail) : 0;
    for (int i = 0; i < 8; i++) {
        char c = (i < (int)taillen) ? t->tail[i] : ' ';
        if (c != ' ' && !(c >= '0' && c <= '9') && !(c >= 'A' && c <= 'Z'))
            c = ' ';
        p[19 + i] = (uint8_t)c;
    }

    // msg[27]: priority / emergency status (M1: none).
    p[27] = 0;
    return 28;
}

// Bounded snapshot buffer (static — too large for the task stack). Sized from
// the table capacity so tuning TRAFFIC_TABLE_MAX can't silently truncate; we
// report all positioned entries each cycle.
static traffic_info_t s_snap[TRAFFIC_TABLE_MAX];

void gdl90_emit_task(void *arg)
{
    (void)arg;
    if (!s_crc_ok) {
        ESP_LOGE(TAG, "CRC self-test failed at init; emitter idle (no frames sent)");
        vTaskDelete(NULL);
    }
    uint8_t payload[512];
    uint8_t frame[1024];

    TickType_t last_hb = xTaskGetTickCount();
    TickType_t last_diag = last_hb;

    for (;;) {
        TickType_t now = xTaskGetTickCount();

        // 1 Hz: heartbeat (0x00), Stratux status (0xCC), and one Traffic Report
        // (0x14) per positioned target.
        if (now - last_hb >= pdMS_TO_TICKS(1000)) {
            int flen;
            size_t positioned = 0;

            size_t plen = build_heartbeat(payload);
            flen = gdl90_frame(payload, plen, frame, sizeof(frame));
            if (flen > 0) net_gdl90_send(frame, (size_t)flen);

            plen = build_stratux_heartbeat(payload);
            flen = gdl90_frame(payload, plen, frame, sizeof(frame));
            if (flen > 0) net_gdl90_send(frame, (size_t)flen);

            size_t n = traffic_snapshot(s_snap, TRAFFIC_TABLE_MAX);
            for (size_t i = 0; i < n; i++) {
                // Position required to plot; traffic_mgr clears position_valid
                // once a fix is stale and no longer being extrapolated, so a
                // frozen position is never re-broadcast as current.
                if (!s_snap[i].position_valid)
                    continue;
                positioned++;
                plen = build_traffic_report(payload, &s_snap[i]);
                flen = gdl90_frame(payload, plen, frame, sizeof(frame));
                if (flen > 0) net_gdl90_send(frame, (size_t)flen);
            }

            // Diagnostic heartbeat every ~5 s: confirms the emitter is alive and
            // shows whether frames are decoding (es_msgs), populating the table,
            // and — critically — whether any EFB lease exists to unicast to.
            if (now - last_diag >= pdMS_TO_TICKS(5000)) {
                ESP_LOGI(TAG, "emit: es_msgs=%lu table=%u positioned=%u assoc=%d leases=%d",
                         (unsigned long)g_status.es_msgs, (unsigned)n,
                         (unsigned)positioned, net_client_count(), net_lease_count());
                last_diag = now;
            }

            last_hb = now;
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
