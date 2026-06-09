// Host (native) unit test for the 1090ES Mode-S decoder.
//
// Compiles the portable decode modules straight from components/modes/src
// (no ESP-IDF) and checks them against the canonical mode-s.org example
// frames. Build & run:
//
//   cc -I components/modes/src -O2 -Wall \
//      tools/test_modes.c \
//      components/modes/src/mode_s_decode.c \
//      components/modes/src/crc.c \
//      components/modes/src/cpr.c \
//      components/modes/src/cpr_store.c \
//      components/modes/src/ais_charset.c \
//      -lm -o /tmp/test_modes && /tmp/test_modes
//
// Exits non-zero on any failed assertion.

#include "mode_s_decode.h"
#include "cpr_store.h"
#include "crc.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

static int g_failures = 0;

#define CHECK(cond, ...) do { \
    if (!(cond)) { printf("FAIL: " __VA_ARGS__); printf("\n"); g_failures++; } \
    else        { printf("ok:   " __VA_ARGS__); printf("\n"); } \
} while (0)

// Same converter as the firmware line parser (exported by mode_s_decode.c).
static int to_bytes(const char *hex, uint8_t *out)
{
    return ms_hex_to_bytes(hex, strlen(hex), out, 14);
}

int main(void)
{
    ms_decode_init();

    uint8_t buf[14];
    ms_msg_t mm;

    // --- Airborne position (CPR even/odd) -- 40621D, lat 52.2572 lon 3.91937 --
    const char *even = "8D40621D58C382D690C8AC2863A7";
    const char *odd  = "8D40621D58C386435CC412692AD6";

    // even frame: CRC must verify, address extracted.
    int n = to_bytes(even, buf);
    CHECK(ms_decode(&mm, buf, n) == 0, "even position frame decodes");
    CHECK(mm.crc_ok, "even frame CRC residual is zero");
    CHECK(mm.addr == 0x40621D, "even frame addr = 40621D (got %06X)", mm.addr);
    CHECK(mm.cpr_valid && mm.cpr_type == MS_CPR_AIRBORNE, "even frame has airborne CPR");
    CHECK(mm.cpr_odd == 0, "even frame is the even CPR frame");
    CHECK(mm.altitude_baro_valid && mm.altitude_baro == 38000,
          "even frame baro altitude = 38000 ft (got %d)", mm.altitude_baro);

    // Feed the older odd frame first, then the newer even frame: with the even
    // frame as the most recent, the global solution is lat 52.2572 lon 3.91937.
    double lat = 0, lng = 0;
    n = to_bytes(odd, buf);
    ms_decode(&mm, buf, n);
    int got_odd = cpr_store_update(0x40621D, &mm, 1000, &lat, &lng);
    CHECK(got_odd == 0, "single odd frame does not resolve a position yet");

    n = to_bytes(even, buf);
    ms_decode(&mm, buf, n);
    int got = cpr_store_update(0x40621D, &mm, 2000, &lat, &lng);
    CHECK(got == 1, "even+odd pair resolves a global position");
    CHECK(fabs(lat - 52.2572) < 0.001, "decoded lat = 52.2572 (got %.5f)", lat);
    CHECK(fabs(lng - 3.91937) < 0.001, "decoded lon = 3.91937 (got %.5f)", lng);

    // --- Airborne velocity (TC19) -- 485020, gs 159 kt, trk 182.88, -832 fpm --
    const char *vel = "8D485020994409940838175B284F";
    n = to_bytes(vel, buf);
    CHECK(ms_decode(&mm, buf, n) == 0, "velocity frame decodes");
    CHECK(mm.addr == 0x485020, "velocity frame addr = 485020 (got %06X)", mm.addr);
    CHECK(mm.gs_valid && fabs(mm.gs - 159.0) < 2.0,
          "ground speed = 159 kt (got %.1f)", mm.gs);
    CHECK(mm.heading_valid && fabs(mm.heading - 182.88) < 1.0,
          "ground track = 182.88 deg (got %.2f)", mm.heading);
    // This frame's vertical-rate source bit (ME bit 36) is 0, so the rate is
    // reported as geometric (dump1090 semantics); modes.c maps either source
    // into the GDL90 vvel field.
    int vrate = mm.baro_rate_valid ? mm.baro_rate
              : mm.geom_rate_valid ? mm.geom_rate : 0;
    CHECK((mm.baro_rate_valid || mm.geom_rate_valid) && vrate == -832,
          "vertical rate = -832 fpm (got %d, %s)", vrate,
          mm.baro_rate_valid ? "baro" : mm.geom_rate_valid ? "geom" : "none");

    // --- Identification (TC4) -- 4840D6, callsign "KLM1023 " ------------------
    const char *ident = "8D4840D6202CC371C32CE0576098";
    n = to_bytes(ident, buf);
    CHECK(ms_decode(&mm, buf, n) == 0, "identification frame decodes");
    CHECK(mm.addr == 0x4840D6, "ident frame addr = 4840D6 (got %06X)", mm.addr);
    CHECK(mm.callsign_valid && strncmp(mm.callsign, "KLM1023", 7) == 0,
          "callsign = KLM1023 (got '%s')", mm.callsign);

    // --- CRC rejection: corrupt a data byte of a good DF17 frame --------------
    n = to_bytes(even, buf);
    buf[5] ^= 0x01; // flip one bit outside the parity field
    CHECK(ms_decode(&mm, buf, n) == -2, "corrupted DF17 frame is rejected (CRC)");

    // --- Mixed-type CPR pair must NOT resolve globally -------------------------
    // Surface frames use a 90-deg CPR encoding, airborne 360-deg; pairing them
    // (routine during takeoff/landing) decodes to garbage. Stage an odd
    // *surface* sample, then feed an even *airborne* frame: no global fix.
    ms_msg_t s;
    memset(&s, 0, sizeof(s));
    n = to_bytes(odd, buf);
    ms_decode(&s, buf, n);
    s.cpr_type = MS_CPR_SURFACE;       // reinterpret the odd frame as surface
    CHECK(cpr_store_update(0x123456, &s, 1000, &lat, &lng) == 0,
          "single odd surface frame does not resolve");
    n = to_bytes(even, buf);
    ms_decode(&mm, buf, n);            // even airborne frame, same aircraft
    CHECK(cpr_store_update(0x123456, &mm, 2000, &lat, &lng) == 0,
          "surface+airborne mixed CPR pair is not decoded globally");
    // A matching airborne odd frame afterwards completes a valid pair again.
    n = to_bytes(odd, buf);
    ms_decode(&mm, buf, n);
    CHECK(cpr_store_update(0x123456, &mm, 3000, &lat, &lng) == 1 &&
          fabs(lat - 52.2572) < 0.01,
          "matching airborne pair still resolves after a mixed sample");

    // --- DF18 CF4 (TIS-B/ADS-R management) carries no traffic — rejected ------
    // Build a syntactically valid DF18 CF4 frame: zero parity, then write the
    // CRC residual into the parity bytes so the residual becomes zero.
    memset(buf, 0, 14);
    buf[0] = (18 << 3) | 4;            // DF18, CF4
    {
        uint32_t c = modesChecksum(buf, 112);
        buf[11] = (uint8_t)(c >> 16);
        buf[12] = (uint8_t)(c >> 8);
        buf[13] = (uint8_t)c;
    }
    CHECK(ms_decode(&mm, buf, 14) == -1, "DF18 CF4 management frame is rejected");

    // --- ms_hex_to_bytes: firmware and tools share one strict converter --------
    CHECK(ms_hex_to_bytes("8D4", 3, buf, 14) == -1, "odd-length hex is rejected");
    CHECK(ms_hex_to_bytes("8G", 2, buf, 14) == -1, "non-hex chars are rejected");
    CHECK(ms_hex_to_bytes("8d4840", 6, buf, 14) == 3 && buf[1] == 0x48,
          "lowercase hex converts");

    printf("\n%s (%d failure%s)\n", g_failures ? "TESTS FAILED" : "ALL TESTS PASSED",
           g_failures, g_failures == 1 ? "" : "s");
    return g_failures ? 1 : 0;
}
