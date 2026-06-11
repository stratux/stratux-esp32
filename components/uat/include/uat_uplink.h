#pragma once
#include <stdint.h>
#include "pong_frame.h"

// FIS-B uplink ('+' line) product tally for the web UI's UAT breakdown (M2).
// Walks the ground-uplink info frames and counts APDUs per 11-bit product ID;
// no product payloads are decoded (that is M5/wxstore). Frame layout per
// Stratux uatparse/uatparse.go, mirrored host-side by tools/fisb_time.py.

#define UAT_UPLINK_PRODUCT_SLOTS 24   // distinct product IDs tracked (FIS-B
                                      // broadcasts ~a dozen in practice)

typedef struct {
    uint16_t product_id;
    uint32_t count;
    int64_t  last_ms;        // caller clock (esp_timer ms) of the last APDU
} fisb_product_stat_t;

typedef struct {
    uint32_t frames;         // uplink frames walked (app data valid)
    uint32_t no_app_data;    // frames with the app-data-valid bit clear
    uint32_t bad_frames;     // undecodable hex / implausibly short lines
    uint32_t info_frames;    // FIS-B APDU info frames tallied
    uint32_t tisb_frames;    // info frames with frame_type != 0 (TIS-B et al.)
    uint32_t overflow;       // APDUs dropped because the product table was full
    fisb_product_stat_t products[UAT_UPLINK_PRODUCT_SLOTS];
} uat_uplink_stats_t;

// Tally one '+' uplink frame. Single writer (the pong task); now_ms is the
// caller's millisecond clock.
void uat_uplink_tally(const pong_frame_t *f, int64_t now_ms);

// Copy the current stats. Best-effort snapshot (see stratux_status.h on why
// unlocked counter reads are acceptable for a status pane).
void uat_uplink_get_stats(uat_uplink_stats_t *out);

// Human-readable product name (Stratux gen_gdl90.go product_name_map);
// returns "Unknown" for IDs not in the table — callers show the raw ID too.
const char *fisb_product_name(uint16_t id);
