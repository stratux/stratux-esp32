#pragma once
#include <stdint.h>
#include <stdbool.h>

// Firmware version string, surfaced in /getStatus and the GDL90 ident.
#define STRATUX_ESP32_VERSION "0.0.0-pre-m0"

// Cross-cutting runtime status snapshot, surfaced by the web UI (GET /getStatus)
// and the Stratux 0xCC heartbeat. Producing tasks update their own fields; the
// web/GDL90 readers only read. Counters are best-effort (no exact-once needed
// for a status pane), so plain reads/writes are acceptable on the ESP32.
typedef struct {
    bool     pong_connected;   // Pong heartbeat/frame seen within the link timeout
    uint32_t es_msgs;          // 1090ES frames decoded since boot
    uint32_t es_rejected;      // 1090ES frames rejected (CRC fail / unknown DF) —
                               // distinguishes a degraded RF path from empty air
    uint32_t uat_msgs;         // UAT frames decoded since boot
    uint32_t pong_errors;      // "ERROR SPI" + line-parse failures
    bool     utc_ok;           // real time source acquired (GPS/NTP/manual) — false until M3
    uint32_t secs_since_midnight; // seconds since 0000Z; only meaningful when utc_ok (M3)
} stratux_status_t;

// Single process-wide instance (defined in stratux_status.c).
extern stratux_status_t g_status;
