#include "uat.h"
#include <string.h>
#include "esp_log.h"
#include "stratux_status.h"
#include "traffic.h"
#include "uat_decode.h"

static const char *TAG = "uat";

// DO-282 address qualifier -> semantic traffic address type. GDL90's
// address-type nibble descends from the UAT MASPS, so this is 1:1 for the
// defined values; reserved qualifiers fall back to "self-assigned" rather
// than emitting a reserved wire value.
static traffic_addr_type_t map_addr_type(address_qualifier_t aq)
{
    switch (aq) {
    case AQ_ADSB_ICAO:    return ADDR_TYPE_ADSB_ICAO;
    case AQ_NATIONAL:     return ADDR_TYPE_ADSB_OTHER;
    case AQ_TISB_ICAO:    return ADDR_TYPE_TISB_ICAO;
    case AQ_TISB_OTHER:   return ADDR_TYPE_TISB_OTHER;
    case AQ_VEHICLE:      return ADDR_TYPE_ADSB_VEHICLE;
    case AQ_FIXED_BEACON: return ADDR_TYPE_FIXED_BEACON;
    default:              return ADDR_TYPE_ADSB_OTHER;
    }
}

// MDB types whose payload extends past the 18-byte short frame (MS and/or
// AUXSV per the dispatch in uat_decode_adsb_mdb). A short frame claiming one
// of these is corrupt — the decoder would read zero padding as field data.
static bool mdb_type_needs_long(uint8_t mdb_type)
{
    switch (mdb_type) {
    case 1: case 2: case 3: case 5: case 6:
        return true;
    default:
        return false;
    }
}

void uat_decode_frame(const pong_frame_t *f)
{
    if (!f || f->hex_len == 0)
        return;

    if (f->kind == PONG_LINE_UAT_UP) {
        // TODO(M5): uplink frames ('+') -> FIS-B products (wxstore) or 0x07
        // relay. Counted so the web UI can show ground-station reception.
        g_status.uat_msgs++;
        return;
    }

    // Zero-padded long buffer: MS/AUXSV reads can never run past a short
    // frame (dump978's demodulator guarantees sizes; a serial line doesn't).
    uint8_t buf[UAT_LONG_FRAME_BYTES] = {0};
    int nbytes = uat_hex_to_bytes(f->hex, f->hex_len, buf, sizeof(buf));
    if (nbytes != UAT_SHORT_FRAME_BYTES && nbytes != UAT_LONG_FRAME_BYTES) {
        g_status.uat_rejected++;
        return;
    }

    struct uat_adsb_mdb mdb;
    uat_decode_adsb_mdb(buf, &mdb);

    if (nbytes == UAT_SHORT_FRAME_BYTES && mdb_type_needs_long(mdb.mdb_type)) {
        g_status.uat_rejected++;
        return;
    }

    g_status.uat_msgs++;

    traffic_info_t t;
    memset(&t, 0, sizeof(t));
    t.icao_addr = mdb.address;
    t.addr_type = map_addr_type(mdb.address_qualifier);

    if (mdb.position_valid) {
        t.position_valid = true;
        t.lat = mdb.lat;
        t.lng = mdb.lon;
    }

    // SV altitude is baro or geo per the frame; a long frame's AUXSV carries
    // the other kind (sec_altitude_type is the inverse of the primary), so
    // one frame can fill both fields.
    if (mdb.altitude_type == ALT_BARO) {
        t.alt_ft = mdb.altitude;
        t.alt_valid = true;
    } else if (mdb.altitude_type == ALT_GEO) {
        t.gnss_alt_ft = mdb.altitude;
        t.gnss_alt_valid = true;
    }
    if (mdb.has_auxsv) {
        if (mdb.sec_altitude_type == ALT_BARO && !t.alt_valid) {
            t.alt_ft = mdb.sec_altitude;
            t.alt_valid = true;
        } else if (mdb.sec_altitude_type == ALT_GEO && !t.gnss_alt_valid) {
            t.gnss_alt_ft = mdb.sec_altitude;
            t.gnss_alt_valid = true;
        }
    }

    // AG_RESERVED leaves the state unknown — don't flip a known on-ground
    // target (same rule as the Mode-S MS_AG_UNCERTAIN handling).
    if (mdb.airground_state == AG_GROUND) {
        t.on_ground = true;
        t.airground_valid = true;
    } else if (mdb.airground_state == AG_SUBSONIC ||
               mdb.airground_state == AG_SUPERSONIC) {
        t.airground_valid = true;
    }

    // Velocity: airborne frames give NS/EW -> speed+track jointly; ground
    // frames give ground speed and a track/heading whose validity is
    // separate. GDL90 has a single speed+track pair, so require both.
    if (mdb.speed_valid && mdb.track_type != TT_INVALID) {
        t.speed_kt = mdb.speed;
        t.track_deg = mdb.track % 360;
        t.speed_valid = true;
    }
    if (mdb.vert_rate_source != ALT_INVALID) {
        t.vvel_fpm = mdb.vert_rate;
        t.vvel_valid = true;
    }

    // CSID gates the base-40 characters: CS_SQUAWK means a Mode-A code —
    // never a tail. (No squawk field in traffic_info_t; it is dropped.)
    if (mdb.has_ms && mdb.callsign_type == CS_CALLSIGN && mdb.callsign[0]) {
        memcpy(t.tail, mdb.callsign, sizeof(t.tail));
        t.tail[sizeof(t.tail) - 1] = '\0';
        t.tail_valid = true;
    }
    if (mdb.has_ms) {
        // UAT MS emitter category is already the flat GDL90 0-39 encoding
        // (GDL90's table descends from DO-282) — no conversion.
        t.emitter_cat = mdb.emitter_category;
        t.cat_valid = true;
        // NACp 0 means "unknown" — leaving it invalid lets gdl90_out seed
        // accuracy from NIC instead of pinning it to 0 forever.
        if (mdb.nac_p > 0) {
            t.nacp = mdb.nac_p;
            t.nacp_valid = true;
        }
    }
    if (mdb.has_sv) {
        t.nic = mdb.nic;
        t.nic_valid = true;
    }

    if (f->ss_valid) {
        t.ss = f->ss;
        t.ss_valid = true;
    }

    // Only report frames that contributed something the traffic table can
    // use (same gate as the Mode-S path).
    if (!(t.position_valid || t.alt_valid || t.gnss_alt_valid || t.speed_valid ||
          t.vvel_valid || t.tail_valid || t.cat_valid))
        return;

    traffic_upsert(&t);

    ESP_LOGV(TAG, "UAT mdb%u %06lX aq=%d pos=%d alt=%ld",
             mdb.mdb_type, (unsigned long)t.icao_addr, (int)mdb.address_qualifier,
             (int)t.position_valid, (long)(t.alt_valid ? t.alt_ft : -9999));
}
