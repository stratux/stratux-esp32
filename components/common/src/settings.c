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
    // sta_en/sta_ssid/sta_pass/gdl90_dest stay zeroed: client mode off.
}

void settings_load(void)
{
    apply_defaults(&g_settings);

    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READONLY, &h) != ESP_OK) {
        ESP_LOGI(TAG, "no saved settings; using defaults (SSID=%s)", g_settings.wifi_ssid);
        return;
    }

    // Each key overrides its default only when present; ESP_ERR_NVS_NOT_FOUND
    // keeps the default already in g_settings. Booleans are stored as u8.
    size_t len;
    uint8_t u8;

    len = sizeof(g_settings.wifi_ssid);
    nvs_get_str(h, "wifi_ssid", g_settings.wifi_ssid, &len);
    len = sizeof(g_settings.wifi_pass);
    nvs_get_str(h, "wifi_pass", g_settings.wifi_pass, &len);
    nvs_get_u8(h, "wifi_chan", &g_settings.wifi_chan);
    if (nvs_get_u8(h, "pong_en", &u8) == ESP_OK) g_settings.pong_en = u8;
    if (nvs_get_u8(h, "es_en",   &u8) == ESP_OK) g_settings.es_en = u8;
    if (nvs_get_u8(h, "uat_en",  &u8) == ESP_OK) g_settings.uat_en = u8;
    nvs_get_u32(h, "ownship", &g_settings.ownship_modes);
    nvs_get_i32(h, "alt_off", &g_settings.alt_off);
    len = sizeof(g_settings.region);
    nvs_get_str(h, "region", g_settings.region, &len);
    if (nvs_get_u8(h, "sta_en", &u8) == ESP_OK) g_settings.sta_en = u8;
    len = sizeof(g_settings.sta_ssid);
    nvs_get_str(h, "sta_ssid", g_settings.sta_ssid, &len);
    len = sizeof(g_settings.sta_pass);
    nvs_get_str(h, "sta_pass", g_settings.sta_pass, &len);
    len = sizeof(g_settings.gdl90_dest);
    nvs_get_str(h, "gdl90_dest", g_settings.gdl90_dest, &len);

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

    // Write every field; first error wins but commit/close always run.
    esp_err_t e;
    err = ESP_OK;
    #define SET(call) do { e = (call); if (e != ESP_OK && err == ESP_OK) err = e; } while (0)
    SET(nvs_set_str(h, "wifi_ssid", g_settings.wifi_ssid));
    SET(nvs_set_str(h, "wifi_pass", g_settings.wifi_pass));
    SET(nvs_set_u8 (h, "wifi_chan", g_settings.wifi_chan));
    SET(nvs_set_u8 (h, "pong_en",   g_settings.pong_en));
    SET(nvs_set_u8 (h, "es_en",     g_settings.es_en));
    SET(nvs_set_u8 (h, "uat_en",    g_settings.uat_en));
    SET(nvs_set_u32(h, "ownship",   g_settings.ownship_modes));
    SET(nvs_set_i32(h, "alt_off",   g_settings.alt_off));
    SET(nvs_set_str(h, "region",    g_settings.region));
    SET(nvs_set_u8 (h, "sta_en",    g_settings.sta_en));
    SET(nvs_set_str(h, "sta_ssid",  g_settings.sta_ssid));
    SET(nvs_set_str(h, "sta_pass",  g_settings.sta_pass));
    SET(nvs_set_str(h, "gdl90_dest", g_settings.gdl90_dest));
    #undef SET

    e = nvs_commit(h);
    if (e != ESP_OK && err == ESP_OK) err = e;
    nvs_close(h);
    return err;
}
