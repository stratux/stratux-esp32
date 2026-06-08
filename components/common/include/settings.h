#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

// NVS-backed configuration — replaces the Pi's /boot/.../stratux.conf JSON.
// Read/written via the web UI (M2). NVS namespace: "stratux".
typedef struct {
    char     wifi_ssid[33];   // NVS "wifi_ssid"  (WiFiSSID)        default "stratux"
    char     wifi_pass[65];   // NVS "wifi_pass"  (WiFiPassphrase)  default "" (open)
    uint8_t  wifi_chan;       // NVS "wifi_chan"  (WiFiChannel)     default 1
    bool     pong_en;         // NVS "pong_en"    (Pong_Enabled)    default true
    bool     es_en;           // NVS "es_en"      (ES_Enabled)      default true
    bool     uat_en;          // NVS "uat_en"     (UAT_Enabled)     default true
    uint32_t ownship_modes;   // NVS "ownship"    (OwnshipModeS)    default 0 (unset)
    int32_t  alt_off;         // NVS "alt_off"    (AltitudeOffset)  default 0
    char     region[4];       // NVS "region"     (RegionSelected)  default "US"
} settings_t;

// Process-wide cached config. Populated by settings_load(); the web UI mutates
// it and calls settings_save(). Treat as read-mostly after boot.
extern settings_t g_settings;

// Load g_settings from NVS, filling in defaults for any missing key.
void settings_load(void);

// Persist the current g_settings back to NVS. Returns ESP_OK on success.
esp_err_t settings_save(void);
