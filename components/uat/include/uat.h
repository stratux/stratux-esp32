#pragma once
#include "pong_frame.h"   // pong_frame_t (in `common`)

// 978 UAT frame decoder. Draws on dump978's message layer +
// Stratux uatparse/. Confirm licensing before vendoring (§13).
//   - M1: decode UAT DOWNLINK ('-') -> traffic_info_t -> traffic_upsert().
//   - M5: decode UAT UPLINK ('+')  -> FIS-B (METAR/TAF/NEXRAD) + wxstore, OR
//         relay raw uplink as GDL90 0x07 passthrough (M5a, easier — see §12).

// Decode one UAT frame (downlink or uplink) from the Pong. Called by the pong
// task (synchronously) for each classified '-' / '+' line.
void uat_decode_frame(const pong_frame_t *f);
