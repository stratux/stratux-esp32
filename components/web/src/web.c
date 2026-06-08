#include "web.h"
#include <stdio.h>
#include "esp_log.h"
#include "esp_http_server.h"
#include "stratux_status.h"
#include "net.h"

static const char *TAG = "web";

// GET /getStatus -> JSON snapshot.
static esp_err_t status_get(httpd_req_t *req)
{
    char buf[256];
    int n = snprintf(buf, sizeof buf,
        "{\"version\":\"%s\",\"pong\":%d,\"utc_ok\":%d,"
        "\"es\":%u,\"uat\":%u,\"pong_err\":%u,\"clients\":%d}",
        STRATUX_ESP32_VERSION, g_status.pong_connected, g_status.utc_ok,
        (unsigned)g_status.es_msgs, (unsigned)g_status.uat_msgs,
        (unsigned)g_status.pong_errors, net_client_count());
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, buf, n);
}

// WS /traffic -> push TrafficInfo JSON frames to subscribers (M2).
static esp_err_t traffic_ws(httpd_req_t *req)
{
    if (req->method == HTTP_GET) return ESP_OK;   // handshake
    // TODO(M2): httpd_ws_recv_frame / push traffic_snapshot() rows via
    // httpd_ws_send_frame to subscribed clients.
    return ESP_OK;
}

void web_start(void)
{
    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.uri_match_fn = httpd_uri_match_wildcard;   // serve /www/* from FAT (M2)

    httpd_handle_t s = NULL;
    if (httpd_start(&s, &cfg) != ESP_OK) {
        ESP_LOGE(TAG, "httpd_start failed");
        return;
    }

    httpd_uri_t u_status  = { .uri = "/getStatus", .method = HTTP_GET,
                              .handler = status_get };
    httpd_uri_t u_traffic = { .uri = "/traffic",   .method = HTTP_GET,
                              .handler = traffic_ws, .is_websocket = true };
    httpd_register_uri_handler(s, &u_status);
    httpd_register_uri_handler(s, &u_traffic);

    // TODO(M2): /getSettings, /setSettings, /getTraffic, WS /status, and a
    // wildcard handler serving static assets from the FAT `storage` partition.
    ESP_LOGI(TAG, "web UI up (http://192.168.10.1/)");
}
