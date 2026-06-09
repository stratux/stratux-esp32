#pragma once
#include "pong_frame.h"   // pong_frame_t (in `common`)

// 1090ES Mode-S (DF17/18) message decoder — NET-NEW (see AGENTS.md M1).
//
// The Pong delivers already-demodulated frames, so we do NOT need dump1090's
// DSP front-end — only its *message-decode* layer. "Minimal" is deceptive: an
// M1-complete decoder needs CRC/parity, ICAO + non-ICAO addressing, even/odd
// CPR pairing with expiry, airborne AND surface position, velocity (TC 19),
// identity/callsign (TC 1-4), NIC/NACp, and DF18 CF handling for ADS-R/TIS-B.
//
// Recommendation: port dump1090's mode_s.c message layer rather than hand-roll
// (Stratux's traffic path assumes dump1090-decoded fields). Confirm licensing
// before vendoring (see AGENTS.md "Conventions" — licensing).

// Build the Mode-S CRC table and clear the ICAO address filter. Call once from
// app_main before starting pong_rx_task.
void modes_init(void);

// Decode one '*' 1090ES frame and traffic_upsert() the result. Called by the
// pong task (synchronously) for each classified 1090ES line.
void modes_decode_frame(const pong_frame_t *f);
