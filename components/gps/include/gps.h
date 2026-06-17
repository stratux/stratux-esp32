#pragma once
#include <stdint.h>
#include <stdbool.h>

// GPS ownship state. Read by gdl90_emit_task for 0x0A/0x0B messages.
// Written only by gps_rx_task; consumed by gdl90 layer and web UI.
typedef struct {
    bool     valid;           // true if fix is current (< 5 sec old)
    double   lat, lng;        // degrees
    int32_t  alt_ft;          // altitude above WGS-84 ellipsoid (geometric), feet
    uint16_t track_deg;       // track true, 0..359
    uint16_t speed_kt;        // ground speed, knots
    int16_t  vvel_fpm;        // vertical velocity, ft/min
    int64_t  fix_time_ms;     // esp_timer_get_time() / 1000 when this fix was acquired
    uint8_t  num_sats;        // satellites in fix
    uint8_t  hdop_x10;        // HDOP * 10 (0..999 maps to 0.0..99.9)
} gps_ownship_t;

// Get the current ownship state (read-only snapshot for gdl90_emit_task).
gps_ownship_t gps_get_ownship(void);

// FreeRTOS task: read BN-220 UART (GPIO 34 RX / GPIO 4 TX, configurable baud),
// parse GGA/RMC NMEA sentences, update ownship state, sync system time on first fix.
// Automatically configures GPS module on startup via MTK commands.
void gps_rx_task(void *arg);
