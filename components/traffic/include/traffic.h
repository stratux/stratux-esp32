#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

// Traffic table — ported/adapted from Stratux main/traffic.go `TrafficInfo`,
// reduced for M1. The decoders (modes_/uat_decode) fill one of these and call
// traffic_upsert(); traffic_mgr_task ages/extrapolates; gdl90_emit + the web UI
// read snapshots. See AGENTS.md (architecture & task map).

typedef enum {
    ADDR_TYPE_ADSB_ICAO = 0,   // DF17 / ES with ICAO address
    ADDR_TYPE_ADSB_OTHER,      // non-ICAO address
    ADDR_TYPE_TISB_ICAO,       // DF18 TIS-B, ICAO
    ADDR_TYPE_TISB_OTHER,      // DF18 TIS-B / ADS-R, non-ICAO
    ADDR_TYPE_UAT,             // 978 UAT downlink
} traffic_addr_type_t;

typedef struct {
    uint32_t            icao_addr;      // 24-bit address (ICAO or non-ICAO)
    traffic_addr_type_t addr_type;
    char                tail[9];        // callsign / tail (TC 1-4), NUL-terminated

    bool                position_valid; // a CPR pair (or UAT pos) has resolved
    double              lat, lng;       // degrees
    int32_t             alt_ft;         // pressure altitude, feet
    int32_t             gnss_alt_ft;    // geometric altitude, feet (if known)
    bool                on_ground;

    uint16_t            track_deg;      // 0..359 true
    uint16_t            speed_kt;       // ground speed
    int16_t             vvel_fpm;       // vertical rate, ft/min

    uint8_t             nic, nacp;      // integrity / accuracy
    uint16_t            ss;             // last signal strength (raw Pong hex reading)

    int64_t             last_seen_ms;   // esp_timer ms at last update
    // CPR even/odd staging is decoder-owned (see modes/); not exposed here.
} traffic_info_t;

// Create the table + its mutex. Call once before starting the tasks.
void traffic_init(void);

// Insert or merge a decoded report (decoders call this under no lock; the
// function takes the table mutex internally). Cross-band dedup is M2.
void traffic_upsert(const traffic_info_t *t);

// FreeRTOS task (~10 Hz, Stratux sendTrafficUpdates): extrapolate dead-reckoned
// positions, age out stale entries, compute bearing/distance from ownship.
void traffic_mgr_task(void *arg);

// Copy up to `max` live entries into `out`; returns the count written. Takes
// and releases the table mutex. Used by gdl90_emit and the web UI.
size_t traffic_snapshot(traffic_info_t *out, size_t max);
