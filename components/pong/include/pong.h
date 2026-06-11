#pragma once
#include <stddef.h>
#include "pong_frame.h"   // pong_frame_t / pong_line_kind_t (in `common`)

// FreeRTOS task (Stratux pongSerialReader): configure UART2 @ 3 Mbaud 8N1, set
// the static RTS level once (ClearRTS equivalent — NOT HW flow control), read
// newline-delimited lines, classify by first character, strip ss=/rs=, and
// decode each frame (1090ES -> modes, UAT -> uat).
//
// Decode is synchronous in this task for the skeleton; if 3 Mbaud bursts cause
// UART backpressure, hand frames to a queue-fed decode task instead (the data
// flow allows for that). On "ERROR SPI" the task must perform
// real recovery (reopen UART / toggle a reset GPIO / mark degraded) — Stratux
// merely logs it (Appendix A).
void pong_rx_task(void *arg);

// Raw-Pong diagnostic ring (M2): every received line (any kind, prefix intact,
// long FIS-B lines truncated) is kept in a small ring for the web UI's
// /getPongLog view — the bring-up debugging seam for the real radio. Renders
// the ring oldest-first as "<seq> <line>\n" rows into out (NUL-terminated);
// returns the byte count written.
size_t pong_diag_copy(char *out, size_t cap);
