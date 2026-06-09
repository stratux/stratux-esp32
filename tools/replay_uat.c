// Host replay harness: feed captured Pong '-' (978 UAT downlink) lines through
// the decoder and print aggregate stats. Mirrors the on-device pipeline (line
// parse rules included — body before ';', rs= decimal, ss= hex int8) minus the
// traffic table / GDL90 layers, so a capture can be validated with no radio.
// dump978's sample-data.txt.gz (gunzipped) works as input. Build:
//
//   cc -I components/uat/src -O2 -Wall tools/replay_uat.c \
//      components/uat/src/uat_decode.c -lm -o /tmp/replay_uat
//   /tmp/replay_uat /tmp/uat_downlink.txt
//
// Cross-check totals against dump978's own `uat2text < file`.

#include "uat_decode.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

int main(int argc, char **argv)
{
    if (argc < 2) { fprintf(stderr, "usage: %s <ponglog>\n", argv[0]); return 2; }
    FILE *fp = fopen(argv[1], "r");
    if (!fp) { perror("fopen"); return 2; }

    long n_down = 0, n_badlen = 0, n_badhex = 0, n_typegated = 0, n_decoded = 0;
    long n_pos = 0, n_baro = 0, n_geo = 0, n_ms = 0, n_callsign = 0, n_squawk = 0;
    long n_auxsv = 0, n_speed = 0, n_ss = 0, n_ss_err = 0;
    long mdb_hist[32] = {0};
    long aq_hist[8] = {0};

    char line[2048];
    while (fgets(line, sizeof(line), fp)) {
        if (line[0] != '-') continue;
        n_down++;

        // Same parse rules as pong.c handle_line().
        char *body = line + 1;
        char *semi = strchr(body, ';');
        size_t hlen = semi ? (size_t)(semi - body) : strcspn(body, "\r\n");
        if (hlen != 36 && hlen != 68) { n_badlen++; continue; }

        for (const char *p = semi; p; p = strchr(p + 1, ';')) {
            if (strncmp(p + 1, "ss=", 3) == 0) {
                unsigned long v = strtoul(p + 4, NULL, 16);
                if ((v & 0xFF) == 0x80) n_ss_err++; else n_ss++;
            }
        }

        uint8_t buf[UAT_LONG_FRAME_BYTES] = {0};
        int nbytes = uat_hex_to_bytes(body, hlen, buf, sizeof(buf));
        if (nbytes != UAT_SHORT_FRAME_BYTES && nbytes != UAT_LONG_FRAME_BYTES) {
            n_badhex++;
            continue;
        }

        struct uat_adsb_mdb mdb;
        uat_decode_adsb_mdb(buf, &mdb);

        // Same short-frame/long-type gate as uat.c.
        if (nbytes == UAT_SHORT_FRAME_BYTES &&
            (mdb.mdb_type == 1 || mdb.mdb_type == 2 || mdb.mdb_type == 3 ||
             mdb.mdb_type == 5 || mdb.mdb_type == 6)) {
            n_typegated++;
            continue;
        }

        n_decoded++;
        mdb_hist[mdb.mdb_type & 31]++;
        aq_hist[mdb.address_qualifier & 7]++;
        if (mdb.position_valid) n_pos++;
        if (mdb.altitude_type == ALT_BARO) n_baro++;
        if (mdb.altitude_type == ALT_GEO) n_geo++;
        if (mdb.has_auxsv && mdb.sec_altitude_type != ALT_INVALID) n_auxsv++;
        if (mdb.speed_valid && mdb.track_type != TT_INVALID) n_speed++;
        if (mdb.has_ms) {
            n_ms++;
            if (mdb.callsign_type == CS_CALLSIGN) n_callsign++;
            if (mdb.callsign_type == CS_SQUAWK) n_squawk++;
        }
    }
    fclose(fp);

    printf("downlink lines:    %ld\n", n_down);
    printf("  bad length:      %ld\n", n_badlen);
    printf("  bad hex:         %ld\n", n_badhex);
    printf("  short/long-type: %ld\n", n_typegated);
    printf("decoded MDBs:      %ld\n", n_decoded);
    for (int i = 0; i < 32; i++)
        if (mdb_hist[i]) printf("  mdb type %-2d      %ld\n", i, mdb_hist[i]);
    for (int i = 0; i < 8; i++)
        if (aq_hist[i]) printf("  addr qual %d      %ld\n", i, aq_hist[i]);
    printf("position valid:    %ld\n", n_pos);
    printf("baro altitude:     %ld\n", n_baro);
    printf("geo altitude:      %ld\n", n_geo);
    printf("auxsv altitude:    %ld\n", n_auxsv);
    printf("speed+track:       %ld\n", n_speed);
    printf("MS frames:         %ld (callsign %ld, squawk %ld)\n",
           n_ms, n_callsign, n_squawk);
    printf("ss= present:       %ld (errored 0x80: %ld)\n", n_ss, n_ss_err);
    return 0;
}
