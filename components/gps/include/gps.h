#pragma once
#include <stdint.h>
#include <stdbool.h>

// GPS ownship state. Read by gdl90_emit_task for 0x0A/0x0B messages.
// Written only by gps_rx_task; consumed by gdl90 layer and web UI.
//
// Altitude semantics (GDL90 needs both): NMEA GGA reports orthometric height
// above mean sea level (field 9) plus the geoid separation (field 11). The
// height above the WGS-84 ellipsoid (HAE, "geometric") is MSL + geoid_sep.
//   - 0x0A Ownship Report carries MSL in the pressure-altitude slot (the EFB
//     fallback when no baro is present)  -> alt_msl_ft
//   - 0x0B Ownship Geometric Altitude carries HAE                    -> alt_hae_ft
typedef struct {
    bool     valid;            // true if fix is current (< 5 sec old)
    double   lat, lng;         // degrees
    int32_t  alt_msl_ft;       // altitude above mean sea level (GGA field 9), feet
    int32_t  alt_hae_ft;       // height above WGS-84 ellipsoid (geometric) = MSL + geoid sep, feet
    uint16_t track_deg;        // track true, 0..359
    uint16_t speed_kt;         // ground speed, knots
    int16_t  vvel_fpm;         // vertical velocity, ft/min (not provided by NMEA; 0)
    int64_t  fix_time_ms;      // esp_timer_get_time() / 1000 when this fix was acquired
    uint8_t  num_sats;         // satellites used in the GGA fix
    uint8_t  num_sats_solution;// satellites used in the GSA solution
    uint8_t  hdop_x10;         // HDOP * 10 (0..999 maps to 0.0..99.9)
    uint8_t  vdop_x10;         // VDOP * 10
    uint8_t  pdop_x10;         // PDOP * 10
    float    geoid_sep_ft;     // geoid separation (HAE - MSL), feet
    uint8_t  fix_quality;      // 0=invalid, 1=GPS, 2=DGPS/WAAS
    uint8_t  nacp;             // NACp (AC 20-165A), derived from HDOP
} gps_ownship_t;

// Get the current ownship state (read-only snapshot for gdl90_emit_task).
gps_ownship_t gps_get_ownship(void);

// FreeRTOS task: read GPS UART (GPIO 34 RX / GPIO 4 TX, configurable baud),
// parse GGA/RMC/GSA NMEA sentences, update ownship state, sync system time on first fix.
// Auto-detects the module baud and configures it via MTK (PMTK) and u-blox UBX.
void gps_rx_task(void *arg);
