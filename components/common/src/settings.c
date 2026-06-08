#include "settings.h"
#include <string.h>
#include "nvs.h"
#include "esp_log.h"

static const char *TAG = "settings";
#define NVS_NS "stratux"

settings_t g_settings;

// Defaults: AP "stratux" on 192.168.10.1, both bands on.
static void apply_defaults(settings_t *s)
{
    memset(s, 0, sizeof(*s));
    strcpy(s->wifi_ssid, "stratux");
    s->wifi_pass[0] = '\0';        // open AP by default
    s->wifi_chan    = 1;
    s->pong_en      = true;
    s->es_en        = true;
    s->uat_en       = true;
    s->ownship_modes = 0;          // unset
    s->alt_off      = 0;
    strcpy(s->region, "US");
}

void settings_load(void)
{
    apply_defaults(&g_settings);

    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READONLY, &h) != ESP_OK) {
        ESP_LOGI(TAG, "no saved settings; using defaults (SSID=%s)", g_settings.wifi_ssid);
        return;
    }

    // TODO(M2): read each key, overriding defaults only when present:
    //   nvs_get_str(h, "wifi_ssid", ...), nvs_get_u8(h, "wifi_chan", ...),
    //   nvs_get_u8(h, "pong_en", ...), nvs_get_u32(h, "ownship", ...), etc.
    //   Booleans are stored as u8. Missing keys (ESP_ERR_NVS_NOT_FOUND) keep
    //   the default already in g_settings.

    nvs_close(h);
    ESP_LOGI(TAG, "settings loaded (SSID=%s, chan=%u, es=%d uat=%d)",
             g_settings.wifi_ssid, g_settings.wifi_chan,
             g_settings.es_en, g_settings.uat_en);
}

esp_err_t settings_save(void)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NS, NVS_READWRITE, &h);
    if (err != ESP_OK) return err;

    // TODO(M2): write each field with nvs_set_str / nvs_set_u8 / nvs_set_u32,
    // then nvs_commit(). Called from the web /setSettings handler.

    err = nvs_commit(h);
    nvs_close(h);
    return err;
}
