// Host replay harness: feed captured Pong '+' (UAT uplink) lines through the
// FIS-B product tally and print the per-product histogram. Validates
// components/uat/src/uat_uplink.c against a real capture with no radio. Build:
//
//   cc -I components/uat/include -I components/uat/src \
//      -I components/common/include -O2 -Wall \
//      tools/replay_uplink.c components/uat/src/uat_uplink.c \
//      components/uat/src/uat_decode.c -lm -o /tmp/replay_uplink
//   /tmp/replay_uplink ponglog-07062025.log
//
// Cross-check: tools/fisb_time.py walks the same info-frame layout; a Python
// product histogram over the same capture must agree on APDU counts.

#include "uat_uplink.h"
#include <stdio.h>
#include <string.h>

int main(int argc, char **argv)
{
    if (argc < 2) { fprintf(stderr, "usage: %s <ponglog>\n", argv[0]); return 2; }
    FILE *fp = fopen(argv[1], "r");
    if (!fp) { perror("fopen"); return 2; }

    char line[2048];
    long n_lines = 0;
    int64_t now_ms = 0;

    while (fgets(line, sizeof(line), fp)) {
        if (line[0] != '+') continue;
        n_lines++;
        now_ms += 20;   // synthetic clock; only 'age' depends on it

        pong_frame_t f = { .kind = PONG_LINE_UAT_UP };
        const char *body = line + 1;
        const char *semi = strchr(body, ';');
        size_t hlen = semi ? (size_t)(semi - body) : strcspn(body, "\r\n");
        if (hlen >= PONG_HEX_MAX) hlen = PONG_HEX_MAX - 1;
        memcpy(f.hex, body, hlen);
        f.hex[hlen] = '\0';
        f.hex_len = (uint16_t)hlen;

        uat_uplink_tally(&f, now_ms);
    }
    fclose(fp);

    uat_uplink_stats_t st;
    uat_uplink_get_stats(&st);

    printf("uplink (+) lines:  %ld\n", n_lines);
    printf("  walked frames:   %u\n", st.frames);
    printf("  no app data:     %u\n", st.no_app_data);
    printf("  bad hex/short:   %u\n", st.bad_frames);
    printf("  FIS-B APDUs:     %u\n", st.info_frames);
    printf("  TIS-B frames:    %u\n", st.tisb_frames);
    printf("  overflow:        %u\n", st.overflow);
    printf("products:\n");
    for (int i = 0; i < UAT_UPLINK_PRODUCT_SLOTS; i++) {
        const fisb_product_stat_t *p = &st.products[i];
        if (!p->count) continue;
        printf("  %4u  %-16s %6u\n", p->product_id,
               fisb_product_name(p->product_id), p->count);
    }
    return 0;
}
