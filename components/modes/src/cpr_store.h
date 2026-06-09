// Per-aircraft CPR even/odd staging for the 1090ES decoder (stratux-esp32).
//
// The traffic table intentionally does not hold CPR state ("decoder-owned"),
// so the Mode-S layer keeps its own small store: the most recent even and odd
// CPR frames per ICAO address (with a 10 s pairing window) plus the last
// resolved position (for single-frame relative decode). It drives the vendored
// dump1090 cpr.c global/relative decoders.

#pragma once
#include <stdint.h>
#include "mode_s_decode.h"

// Feed a decoded position message (mm->cpr_valid must be set) into the store.
// now_ms is a monotonic millisecond clock. On a resolved fix returns 1 and
// writes *out_lat/*out_lon (degrees); otherwise returns 0.
//
// Airborne positions resolve globally from a fresh even/odd pair (no reference
// needed). Surface positions and single-frame relative decode need a prior
// resolved position as reference; until ownship GPS exists (M3) surface fixes
// generally won't resolve.
int cpr_store_update(uint32_t addr, const ms_msg_t *mm, int64_t now_ms,
                     double *out_lat, double *out_lon);
