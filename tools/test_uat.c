// Host (native) unit test for the 978 UAT downlink decoder.
//
// Compiles the portable decode module straight from components/uat/src
// (no ESP-IDF) and checks it against frames from dump978's
// sample-data.txt.gz, with expected values transcribed from dump978's
// own uat2text output (the ground truth). Build & run:
//
//   cc -I components/uat/src -O2 -Wall \
//      tools/test_uat.c components/uat/src/uat_decode.c \
//      -lm -o /tmp/test_uat && /tmp/test_uat
//
// Exits non-zero on any failed assertion.

#include "uat_decode.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

static int g_failures = 0;

#define CHECK(cond, ...) do { \
    if (!(cond)) { printf("FAIL: " __VA_ARGS__); printf("\n"); g_failures++; } \
    else        { printf("ok:   " __VA_ARGS__); printf("\n"); } \
} while (0)

// Decode one hex frame the same way uat.c does: zero-padded long buffer so
// MS/AUXSV reads can never run past a short frame.
static int decode(const char *hex, struct uat_adsb_mdb *mdb)
{
    uint8_t buf[UAT_LONG_FRAME_BYTES] = {0};
    int n = uat_hex_to_bytes(hex, strlen(hex), buf, sizeof(buf));
    if (n != UAT_SHORT_FRAME_BYTES && n != UAT_LONG_FRAME_BYTES)
        return -1;
    uat_decode_adsb_mdb(buf, mdb);
    return n;
}

static void test_hex_helper(void)
{
    uint8_t buf[4];
    CHECK(uat_hex_to_bytes("00a6", 4, buf, 4) == 2 && buf[0] == 0x00 && buf[1] == 0xa6,
          "hex helper: basic conversion");
    CHECK(uat_hex_to_bytes("0a6", 3, buf, 4) == -1, "hex helper: odd length rejected");
    CHECK(uat_hex_to_bytes("", 0, buf, 4) == -1, "hex helper: empty rejected");
    CHECK(uat_hex_to_bytes("0g", 2, buf, 4) == -1, "hex helper: non-hex rejected");
    CHECK(uat_hex_to_bytes("aAbB", 4, buf, 4) == 2 && buf[0] == 0xaa && buf[1] == 0xbb,
          "hex helper: mixed case");
}

// Short frame, MDB type 0 (HDR + SV), airborne subsonic.
// uat2text: addr A66EF1 (ADS-B/ICAO), NIC 9, +37.4534 -122.0964, 1000 ft baro,
// ns -99 kt, ew +65 kt, track 146, speed 118 kt, vvel -192 ft/min (geo),
// UTC-coupled, no TIS-B site.
static void test_short_type0(void)
{
    struct uat_adsb_mdb m;
    CHECK(decode("00a66ef135445d525a0c0519119021204800", &m) == UAT_SHORT_FRAME_BYTES,
          "type0: decodes as short frame");
    CHECK(m.mdb_type == 0, "type0: mdb_type 0");
    CHECK(m.address_qualifier == AQ_ADSB_ICAO, "type0: AQ ADS-B ICAO");
    CHECK(m.address == 0xA66EF1, "type0: address A66EF1");
    CHECK(m.has_sv && !m.has_ms && !m.has_auxsv, "type0: SV only");
    CHECK(m.nic == 9, "type0: NIC 9");
    CHECK(m.position_valid, "type0: position valid");
    CHECK(fabs(m.lat - 37.4534) < 5e-4 && fabs(m.lon - (-122.0964)) < 5e-4,
          "type0: lat/lon %.4f/%.4f", m.lat, m.lon);
    CHECK(m.altitude_type == ALT_BARO && m.altitude == 1000, "type0: 1000 ft baro");
    CHECK(m.airground_state == AG_SUBSONIC, "type0: airborne subsonic");
    CHECK(m.ns_vel_valid && m.ns_vel == -99, "type0: ns_vel -99 (got %d)", m.ns_vel);
    CHECK(m.ew_vel_valid && m.ew_vel == 65, "type0: ew_vel +65 (got %d)", m.ew_vel);
    CHECK(m.speed_valid && m.speed == 118, "type0: speed 118 (got %u)", m.speed);
    CHECK(m.track_type == TT_TRACK && m.track == 146, "type0: track 146 (got %u)", m.track);
    CHECK(m.vert_rate_source == ALT_GEO && m.vert_rate == -192,
          "type0: vvel -192 geo (got %d)", m.vert_rate);
    CHECK(m.utc_coupled && m.tisb_site_id == 0, "type0: UTC-coupled, no site ID");
}

// Long frame, MDB type 1 (HDR + SV + MS + AUXSV), CSID=1 (real callsign).
// uat2text: NIC 9, +37.4364 -122.0806, 975 ft baro, track 139, speed 128,
// vvel -128 geo; MS: category 2 (Medium Wake 7-34t), callsign N5130E,
// version 2, SIL 3, NACp 10, NACv 2; AUXSV: 1200 ft geometric.
static void test_long_type1_callsign(void)
{
    struct uat_adsb_mdb m;
    CHECK(decode("08a66ef1353e2d525fd4050911882aa038101d06b85d440be2a4c2a0000590000000", &m)
              == UAT_LONG_FRAME_BYTES,
          "type1: decodes as long frame");
    CHECK(m.mdb_type == 1, "type1: mdb_type 1");
    CHECK(m.has_sv && m.has_ms && m.has_auxsv, "type1: SV + MS + AUXSV");
    CHECK(m.address == 0xA66EF1 && m.address_qualifier == AQ_ADSB_ICAO,
          "type1: address A66EF1 ADS-B ICAO");
    CHECK(fabs(m.lat - 37.4364) < 5e-4 && fabs(m.lon - (-122.0806)) < 5e-4,
          "type1: lat/lon %.4f/%.4f", m.lat, m.lon);
    CHECK(m.altitude_type == ALT_BARO && m.altitude == 975, "type1: 975 ft baro");
    CHECK(m.speed_valid && m.speed == 128 && m.track == 139,
          "type1: speed 128 / track 139");
    CHECK(m.vert_rate_source == ALT_GEO && m.vert_rate == -128, "type1: vvel -128 geo");
    CHECK(m.emitter_category == 2, "type1: emitter category 2 (got %u)", m.emitter_category);
    CHECK(m.callsign_type == CS_CALLSIGN && strcmp(m.callsign, "N5130E") == 0,
          "type1: callsign N5130E (got '%s')", m.callsign);
    CHECK(m.uat_version == 2 && m.sil == 3, "type1: version 2, SIL 3");
    CHECK(m.nac_p == 10 && m.nac_v == 2, "type1: NACp 10, NACv 2");
    CHECK(m.sec_altitude_type == ALT_GEO && m.sec_altitude == 1200,
          "type1: AUXSV 1200 ft geo (primary baro -> secondary geo)");
}

// Long frame, MDB type 1 with CSID=0: the base-40 chars are a Mode-A squawk
// (0322), NOT a callsign — the integration layer must not put this in `tail`.
static void test_long_type1_squawk(void)
{
    struct uat_adsb_mdb m;
    CHECK(decode("08a66ef1353ae55263ac04f9117c2ba03f0c830cf5ed2d0bbaa4c0a0000590000000", &m)
              == UAT_LONG_FRAME_BYTES,
          "squawk: decodes as long frame");
    CHECK(m.mdb_type == 1 && m.has_ms, "squawk: type 1 with MS");
    CHECK(m.callsign_type == CS_SQUAWK, "squawk: callsign_type CS_SQUAWK");
    CHECK(strcmp(m.callsign, "0322") == 0, "squawk: code 0322 (got '%s')", m.callsign);
    CHECK(m.nac_p == 10, "squawk: NACp 10");
}

// Long frame, MDB type 2 (HDR + SV + AUXSV, no MS).
// uat2text: 950 ft baro, track 136, speed 128, vvel 0 ft/min (geo, valid),
// AUXSV 1200 ft geometric.
static void test_long_type2(void)
{
    struct uat_adsb_mdb m;
    CHECK(decode("10a66ef1353ced52614204f911782d00180000000000000000000000000590000000", &m)
              == UAT_LONG_FRAME_BYTES,
          "type2: decodes as long frame");
    CHECK(m.mdb_type == 2, "type2: mdb_type 2");
    CHECK(m.has_sv && !m.has_ms && m.has_auxsv, "type2: SV + AUXSV, no MS");
    CHECK(fabs(m.lat - 37.4330) < 5e-4 && fabs(m.lon - (-122.0766)) < 5e-4,
          "type2: lat/lon %.4f/%.4f", m.lat, m.lon);
    CHECK(m.altitude_type == ALT_BARO && m.altitude == 950, "type2: 950 ft baro");
    CHECK(m.vert_rate_source == ALT_GEO && m.vert_rate == 0,
          "type2: vvel 0 ft/min, geo source, still valid");
    CHECK(m.sec_altitude_type == ALT_GEO && m.sec_altitude == 1200,
          "type2: AUXSV 1200 ft geo");
}

// All-zero SV (NIC 0, raw lat/lon 0) must not claim a position.
static void test_zero_position_invalid(void)
{
    struct uat_adsb_mdb m;
    CHECK(decode("000000000000000000000000000000000000", &m) == UAT_SHORT_FRAME_BYTES,
          "zeros: decodes as short frame");
    CHECK(!m.position_valid, "zeros: position invalid");
    CHECK(m.altitude_type == ALT_INVALID, "zeros: altitude invalid");
    CHECK(!m.speed_valid && m.track_type == TT_INVALID, "zeros: velocity invalid");
}

// Frame-size gating (mirrors the uat.c integration rule): only 18 or 34
// byte payloads are dispatchable.
static void test_length_gating(void)
{
    struct uat_adsb_mdb m;
    CHECK(decode("00a66ef1", &m) == -1, "gating: 4-byte frame rejected");
    CHECK(decode("00a66ef135445d525a0c051911902120480000", &m) == -1,
          "gating: 19-byte frame rejected");
}

int main(void)
{
    test_hex_helper();
    test_short_type0();
    test_long_type1_callsign();
    test_long_type1_squawk();
    test_long_type2();
    test_zero_position_invalid();
    test_length_gating();

    printf("\n%s (%d failure%s)\n", g_failures ? "FAILED" : "PASSED",
           g_failures, g_failures == 1 ? "" : "s");
    return g_failures ? 1 : 0;
}
