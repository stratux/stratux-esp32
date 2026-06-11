#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

// Traffic table — ported/adapted from Stratux main/traffic.go `TrafficInfo`,
// reduced for M1. The decoders (modes_/uat_decode) fill one of these and call
// traffic_upsert(); traffic_mgr_task ages/extrapolates; gdl90_emit + the web UI
// read snapshots. See AGENTS.md (architecture & task map).

// Semantic address source. NOT a wire format: gdl90_out maps these to the
// GDL90 address-type nibble explicitly (the ICD has no "UAT" address type).
typedef enum {
    ADDR_TYPE_ADSB_ICAO = 0,   // DF17 / ES or UAT AQ0, ICAO address
    ADDR_TYPE_ADSB_OTHER,      // non-ICAO / self-assigned address
    ADDR_TYPE_TISB_ICAO,       // DF18 TIS-B / ADS-R or UAT AQ2, ICAO
    ADDR_TYPE_TISB_OTHER,      // DF18 TIS-B / ADS-R or UAT AQ3, track file
    ADDR_TYPE_ADSB_VEHICLE,    // UAT AQ4, surface vehicle
    ADDR_TYPE_FIXED_BEACON,    // UAT AQ5, fixed ADS-B beacon
} traffic_addr_type_t;

// Table capacity, shared with consumers that size snapshot buffers (gdl90_out,
// web UI). ~300 * sizeof(traffic_info_t) (~30 KB) comfortably fits internal RAM.
#define TRAFFIC_TABLE_MAX 300

// Receive band of a report. The ICAO-keyed table merges the same aircraft heard
// on both links into one entry; src_bands records which links contributed so
// the web UI can show ES / UAT / ES+UAT.
#define TRAFFIC_SRC_ES  0x01   // 1090ES (Mode-S)
#define TRAFFIC_SRC_UAT 0x02   // 978 UAT downlink

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
    uint8_t             emitter_cat;    // GDL90 emitter category (0-39; TC 1-4)
    uint16_t            ss;             // last signal strength (raw Pong hex reading)
    uint8_t             src;            // TRAFFIC_SRC_* of this report (decoders set)
    uint8_t             src_bands;      // accumulated TRAFFIC_SRC_* mask (table sets)

    int64_t             last_seen_ms;   // esp_timer ms at last update (any frame)
    int64_t             position_ms;    // esp_timer ms of the last real position
                                        // fix — NOT refreshed by non-position
                                        // frames; anchors extrapolation/staleness

    // Per-field validity. A single 1090ES frame carries only some of the state
    // (a velocity message has no altitude, a position message no callsign, …),
    // so the decoder sets only the flags for fields it actually populated and
    // traffic_upsert() merges field-by-field instead of clobbering. On a stored
    // entry, a flag means "this field has been seen at least once".
    bool                alt_valid;
    bool                gnss_alt_valid;
    bool                speed_valid;     // track_deg + speed_kt
    bool                vvel_valid;
    bool                tail_valid;
    bool                cat_valid;
    bool                nic_valid;       // NIC ships with every position report
    bool                nacp_valid;      // NACp only with TC31 operational status
    bool                airground_valid; // on_ground is meaningful
    bool                ss_valid;

    bool                extrapolated;    // position was dead-reckoned by traffic_mgr
    int64_t             extrap_ms;       // time of last extrapolation step (0 = none)
    // CPR even/odd staging is decoder-owned (see modes/); not exposed here.
} traffic_info_t;

// Create the table + its mutex. Call once before starting the tasks.
void traffic_init(void);

// Insert or merge a decoded report (decoders call this under no lock; the
// function takes the table mutex internally). Entries are keyed by ICAO within
// the ICAO-addressed class (so DF17 and a TIS-B/ADS-R rebroadcast of the same
// aircraft merge into one entry, and the same aircraft heard on 1090 and 978
// dedups cross-band); non-ICAO/self-assigned addresses key separately.
void traffic_upsert(const traffic_info_t *t);

// FreeRTOS task (1 Hz, Stratux sendTrafficUpdates): extrapolate dead-reckoned
// positions, demote stale positions, age out stale entries, compute
// bearing/distance from ownship (M3).
void traffic_mgr_task(void *arg);

// Copy up to `max` live entries into `out`; returns the count written. Takes
// and releases the table mutex. Used by gdl90_emit and the web UI.
size_t traffic_snapshot(traffic_info_t *out, size_t max);

// Current live entry count. Takes and releases the table mutex. Lets a capped
// snapshot consumer report how many entries it did NOT copy.
size_t traffic_count(void);
