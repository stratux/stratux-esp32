// Host replay harness: feed captured Pong '*' (1090ES) lines through the
// decoder + CPR store and print aggregate stats. Mirrors the on-device pipeline
// (minus the traffic table / GDL90 layers) so a capture can be validated with
// no radio. Build:
//
//   cc -I components/modes/src -O2 -Wall tools/replay_modes.c \
//      components/modes/src/mode_s_decode.c components/modes/src/crc.c \
//      components/modes/src/cpr.c components/modes/src/cpr_store.c \
//      components/modes/src/ais_charset.c -lm -o /tmp/replay_modes
//   /tmp/replay_modes ponglog-07062025.log

#include "mode_s_decode.h"
#include "cpr_store.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

int main(int argc, char **argv)
{
    if (argc < 2) { fprintf(stderr, "usage: %s <ponglog>\n", argv[0]); return 2; }
    FILE *fp = fopen(argv[1], "r");
    if (!fp) { perror("fopen"); return 2; }

    ms_decode_init();

    char line[2048];
    long n_es = 0, n_decoded = 0, n_crcfail = 0, n_pos = 0, n_vel = 0, n_ident = 0, n_fixes = 0;
    int64_t now_ms = 0;

    while (fgets(line, sizeof(line), fp)) {
        if (line[0] != '*') continue;
        n_es++;
        now_ms++;   // ~1 ms per line; comfortably inside the 10 s CPR window

        const char *body = line + 1;
        const char *semi = strchr(body, ';');
        size_t hlen = semi ? (size_t)(semi - body) : strlen(body);
        if (hlen && (body[hlen - 1] == '\n' || body[hlen - 1] == '\r')) hlen--;
        if (hlen && body[hlen - 1] == '\r') hlen--;
        // Same converter and same rejection rules as the firmware (modes.c),
        // so replay stats predict on-device behavior.
        uint8_t buf[14];
        int nb = ms_hex_to_bytes(body, hlen, buf, sizeof(buf));
        if (nb < 7) continue;

        ms_msg_t mm;
        int rc = ms_decode(&mm, buf, nb);
        if (rc == -2) { n_crcfail++; continue; }
        if (rc != 0) continue;
        n_decoded++;

        if (mm.callsign_valid) n_ident++;
        if (mm.gs_valid && mm.heading_valid) n_vel++;
        if (mm.cpr_valid) {
            n_pos++;
            double lat, lng;
            if (cpr_store_update(mm.addr, &mm, now_ms, &lat, &lng))
                n_fixes++;
        }
    }
    fclose(fp);

    printf("1090ES (*) lines:      %ld\n", n_es);
    printf("  decoded (DF*):       %ld\n", n_decoded);
    printf("  CRC rejected (DF17/18): %ld\n", n_crcfail);
    printf("  position messages:   %ld\n", n_pos);
    printf("  resolved positions:  %ld\n", n_fixes);
    printf("  velocity messages:   %ld\n", n_vel);
    printf("  ident/callsign msgs: %ld\n", n_ident);
    return 0;
}
