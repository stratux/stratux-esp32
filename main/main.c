// stratux-esp32 — app entry point. Brings up NVS, the WiFi SoftAP, the GDL90
// emitter, the traffic table, and the worker tasks. See AGENTS.md for the data
// flow and task map.
#include "nvs_flash.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "settings.h"
#include "net.h"
#include "gdl90_out.h"
#include "traffic.h"
#include "modes.h"
#include "pong.h"
#include "web.h"

static const char *TAG = "stratux";

void app_main(void)
{
    esp_err_t nvs = nvs_flash_init();
    if (nvs == ESP_ERR_NVS_NO_FREE_PAGES || nvs == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        nvs = nvs_flash_init();
    }
    ESP_ERROR_CHECK(nvs);

    settings_load();             // NVS -> g_settings (defaults if unset)
    net_wifi_softap_start();     // SSID "stratux", 192.168.10.1, UDP :4000 sink
    gdl90_out_init();            // CRC self-test + HDLC/CRC framer
    traffic_init();              // table + mutex
    modes_init();                // Mode-S CRC table + ICAO address filter

    // Worker tasks (Go goroutine -> FreeRTOS task; see AGENTS.md task map).
    xTaskCreate(pong_rx_task,     "pong_rx",     4096, NULL, 10, NULL);
    xTaskCreate(traffic_mgr_task, "traffic_mgr", 4096, NULL,  8, NULL);
    xTaskCreate(gdl90_emit_task,  "gdl90_emit",  4096, NULL,  9, NULL);

    web_start();                 // esp_http_server + WS (M2)

    ESP_LOGI(TAG, "stratux-esp32 up");
}
