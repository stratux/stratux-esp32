#include "modes.h"
#include <string.h>
#include <stdbool.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "stratux_status.h"
#include "traffic.h"
#include "mode_s_decode.h"
#include "cpr_store.h"

static const char *TAG = "modes";

// ---- ICAO address filter ---------------------------------------------------
//
// DF0/4/5/16/20/21 use Address/Parity, so their address is *inferred* from the
// CRC syndrome and cannot be verified — accepting them blindly would inject
// phantom targets. dump1090/Stratux gate them behind a set of addresses
// recently seen in CRC-verified frames (DF11/17/18). We do the same with a
// small open-addressed, time-aged hash set.

#define ICAO_FILTER_SIZE   1024            // power of two
#define ICAO_FILTER_MASK   (ICAO_FILTER_SIZE - 1)
#define ICAO_FILTER_TTL_MS 60000
#define ICAO_FILTER_PROBE  16

static uint32_t s_filt_addr[ICAO_FILTER_SIZE];
static int64_t  s_filt_seen[ICAO_FILTER_SIZE];

static inline uint32_t icao_hash(uint32_t a)
{
    return (a * 2654435761u) & ICAO_FILTER_MASK;
}

static void icao_filter_add(uint32_t a, int64_t now_ms)
{
    uint32_t h = icao_hash(a);
    for (int i = 0; i < ICAO_FILTER_PROBE; i++) {
        uint32_t idx = (h + i) & ICAO_FILTER_MASK;
        if (s_filt_addr[idx] == a || s_filt_addr[idx] == 0 ||
            (now_ms - s_filt_seen[idx]) > ICAO_FILTER_TTL_MS) {
            s_filt_addr[idx] = a;
            s_filt_seen[idx] = now_ms;
            return;
        }
    }
    // Probe window full: overwrite the home slot.
    s_filt_addr[h] = a;
    s_filt_seen[h] = now_ms;
}

static bool icao_filter_test(uint32_t a, int64_t now_ms)
{
    uint32_t h = icao_hash(a);
    for (int i = 0; i < ICAO_FILTER_PROBE; i++) {
        uint32_t idx = (h + i) & ICAO_FILTER_MASK;
        if (s_filt_addr[idx] == a)
            return (now_ms - s_filt_seen[idx]) <= ICAO_FILTER_TTL_MS;
        if (s_filt_addr[idx] == 0)
            return false;
    }
    return false;
}

// ---- ms_msg_t -> traffic addr_type ----------------------------------------

static traffic_addr_type_t map_addr_type(const ms_msg_t *mm)
{
    traffic_addr_type_t at = ADDR_TYPE_ADSB_ICAO;

    if (mm->df == 18) {
        switch (mm->cf) {
        case 2: at = ADDR_TYPE_TISB_ICAO;  break; // Fine TIS-B, ICAO
        case 3: at = ADDR_TYPE_TISB_ICAO;  break; // Coarse TIS-B
        case 5: at = ADDR_TYPE_TISB_OTHER; break; // Fine TIS-B, non-ICAO
        case 6: at = ADDR_TYPE_TISB_ICAO;  break; // ADS-R (Stratux addr_type 2)
        case 1: at = ADDR_TYPE_ADSB_OTHER; break; // anonymous / ground vehicle
        default: at = ADDR_TYPE_ADSB_ICAO; break; // CF0 ADS-B NT
        }
    }

    if (mm->non_icao) {
        if (at == ADDR_TYPE_ADSB_ICAO) at = ADDR_TYPE_ADSB_OTHER;
        else if (at == ADDR_TYPE_TISB_ICAO) at = ADDR_TYPE_TISB_OTHER;
    }
    return at;
}

// ---- public API ------------------------------------------------------------

void modes_init(void)
{
    ms_decode_init();
    memset(s_filt_addr, 0, sizeof(s_filt_addr));
    memset(s_filt_seen, 0, sizeof(s_filt_seen));
    ESP_LOGI(TAG, "Mode-S decoder ready");
}

void modes_decode_frame(const pong_frame_t *f)
{
    if (!f || f->hex_len == 0)
        return;

    uint8_t bytes[14];
    int nbytes = ms_hex_to_bytes(f->hex, f->hex_len, bytes, sizeof(bytes));
    if (nbytes < 7) {
        g_status.pong_errors++;
        return;
    }

    ms_msg_t mm;
    if (ms_decode(&mm, bytes, nbytes) != 0) {
        // Bad length / unknown DF / failed CRC. Counted separately so a
        // degraded RF path doesn't look like empty airspace in /getStatus.
        g_status.es_rejected++;
        return;
    }

    g_status.es_msgs++;

    const int64_t now_ms = esp_timer_get_time() / 1000;

    // Address validation. CRC-verified frames with a true ICAO address seed
    // the filter (non-ICAO/self-assigned addresses are not aircraft ICAOs);
    // inferred addresses are only trusted if the filter has seen them recently.
    if (mm.address_reliable) {
        if (!mm.non_icao)
            icao_filter_add(mm.addr, now_ms);
    } else if (!icao_filter_test(mm.addr, now_ms)) {
        return; // unverified address, not a known aircraft — drop
    }

    // DF11 confirms an address but carries no usable traffic payload.
    if (mm.df == 11)
        return;

    traffic_info_t t;
    memset(&t, 0, sizeof(t));
    t.icao_addr = mm.addr;
    t.addr_type = map_addr_type(&mm);

    // Callsign (TC 1-4), trailing spaces trimmed.
    if (mm.callsign_valid) {
        memcpy(t.tail, mm.callsign, sizeof(t.tail));
        t.tail[8] = '\0';
        for (int i = (int)strlen(t.tail) - 1; i >= 0 && t.tail[i] == ' '; i--)
            t.tail[i] = '\0';
        t.tail_valid = true;
    }
    if (mm.category_valid) {
        // mm.category is dump1090's raw set/subtype byte (0xA1 = set A,
        // subtype 1). GDL90 byte 18 wants the flat 0-39 enum: set A = 0-7,
        // B = 8-15, C = 16-23, D = 24-31 (Stratux: "A7 becomes 0x07, B0
        // becomes 0x08"). TC4 is set A … TC1 is set D.
        t.emitter_cat = (uint8_t)(((4 - mm.metype) & 0x03) * 8 + (mm.mesub & 0x07));
        t.cat_valid = true;
    }

    // Position (resolved via the decoder-owned CPR store).
    if (mm.cpr_valid) {
        double lat, lng;
        if (cpr_store_update(mm.addr, &mm, now_ms, &lat, &lng)) {
            t.position_valid = true;
            t.lat = lat;
            t.lng = lng;
        }
    }

    // Altitude.
    if (mm.altitude_baro_valid) {
        t.alt_ft = mm.altitude_baro;
        t.alt_valid = true;
    }
    if (mm.altitude_geom_valid) {
        t.gnss_alt_ft = mm.altitude_geom;
        t.gnss_alt_valid = true;
    }

    // Air/ground. Only a definitive state updates the table: MS_AG_UNCERTAIN
    // (CA=0/6/7 transponders, DF0/16 VS=0) must not flip a known on-ground
    // target back to airborne between surface fixes (dump1090 does the same).
    if (mm.airground == MS_AG_GROUND || mm.airground == MS_AG_AIRBORNE) {
        t.on_ground = (mm.airground == MS_AG_GROUND);
        t.airground_valid = true;
    }

    // Velocity (TC19) / surface movement.
    if (mm.gs_valid && mm.heading_valid) {
        t.speed_kt = (uint16_t)(mm.gs + 0.5f);
        t.track_deg = (uint16_t)(mm.heading + 0.5f) % 360;
        t.speed_valid = true;
    }
    if (mm.baro_rate_valid) {
        t.vvel_fpm = (int16_t)mm.baro_rate;
        t.vvel_valid = true;
    } else if (mm.geom_rate_valid) {
        t.vvel_fpm = (int16_t)mm.geom_rate;
        t.vvel_valid = true;
    }

    // Integrity / accuracy. NIC ships with every position report; NACp only
    // with TC31 operational status — they merge independently in the table.
    // (gdl90_out seeds NACp from NIC at emit time when no real NACp is known.)
    if (mm.nic_valid) {
        t.nic = mm.nic;
        t.nic_valid = true;
    }
    if (mm.nac_p_valid) {
        t.nacp = mm.nac_p;
        t.nacp_valid = true;
    }

    if (f->ss_valid) {
        t.ss = f->ss;
        t.ss_valid = true;
    }

    // Only report frames that contributed something the traffic table can use.
    // DF5/21 (squawk-only — no squawk field in traffic_info_t) and undecoded
    // DF18 formats would otherwise create field-less entries that consume the
    // table and age-out timers without ever being emittable.
    if (!(t.position_valid || t.alt_valid || t.gnss_alt_valid || t.speed_valid ||
          t.vvel_valid || t.tail_valid || t.cat_valid))
        return;

    traffic_upsert(&t);
}
