#include "web.h"
#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "esp_http_server.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "cJSON.h"
#include "settings.h"
#include "stratux_status.h"
#include "net.h"
#include "traffic.h"
#include "uat_uplink.h"
#include "pong.h"

static const char *TAG = "web";

static httpd_handle_t s_server;

// Embedded UI (EMBED_TXTFILES www/index.html — NUL-terminated). Static-asset
// serving from the FAT `storage` partition is deferred: one file doesn't need
// a filesystem, and the current sdkconfig (FATFS_LFN_NONE) couldn't even hold
// "index.html" (8.3 names cap extensions at 3 chars).
extern const uint8_t _binary_index_html_start[];
extern const uint8_t _binary_index_html_end[];

// Snapshot/scratch buffers, static by design: too large for the httpd task
// stack, and every user runs in the single httpd context (work fns + handlers
// are serialized), so one set suffices. Rows are capped — never silently:
// payloads carry "truncated" when the cap bites.
#define WEB_TRAFFIC_ROWS 100
static traffic_info_t s_snap[WEB_TRAFFIC_ROWS];
static char s_row[360];            // one traffic row's JSON
static char s_wsbuf[16 * 1024];    // assembled WS frame
static char s_ponglog[32 * 136];   // rendered diag ring: DIAG_LINES rows of
                                   // up to DIAG_LINE_LEN + "<seq> " + '\n'

// ---- JSON helpers -----------------------------------------------------------

// One traffic row. Invalid fields are emitted as null so the JS needs no
// per-field validity protocol.
static int traffic_row_json(char *buf, size_t cap, const traffic_info_t *t,
                            int64_t now_ms)
{
    size_t u = 0;
    #define APP(...) do { \
        int _w = snprintf(buf + u, cap - u, __VA_ARGS__); \
        if (_w < 0 || (size_t)_w >= cap - u) return -1; \
        u += (size_t)_w; \
    } while (0)

    APP("{\"icao\":\"%06lX\",\"typ\":%d,\"src\":%u",
        (unsigned long)t->icao_addr, (int)t->addr_type, t->src_bands);

    if (t->tail_valid) {
        // Tail is sanitized A-Z/0-9/space at decode; safe to inline into JSON.
        APP(",\"tail\":\"%s\"", t->tail);
    } else {
        APP(",\"tail\":null");
    }
    if (t->cat_valid) APP(",\"cat\":%u", t->emitter_cat); else APP(",\"cat\":null");

    if (t->position_valid)
        APP(",\"lat\":%.5f,\"lng\":%.5f,\"ext\":%d",
            t->lat, t->lng, t->extrapolated ? 1 : 0);
    else
        APP(",\"lat\":null,\"lng\":null,\"ext\":0");

    if (t->alt_valid)      APP(",\"alt\":%ld",  (long)t->alt_ft);      else APP(",\"alt\":null");
    if (t->gnss_alt_valid) APP(",\"galt\":%ld", (long)t->gnss_alt_ft); else APP(",\"galt\":null");
    if (t->airground_valid) APP(",\"gnd\":%d", t->on_ground ? 1 : 0);  else APP(",\"gnd\":null");
    if (t->speed_valid)    APP(",\"spd\":%u,\"trk\":%u", t->speed_kt, t->track_deg);
    else                   APP(",\"spd\":null,\"trk\":null");
    if (t->vvel_valid)     APP(",\"vvel\":%d", (int)t->vvel_fpm);      else APP(",\"vvel\":null");
    if (t->nic_valid)      APP(",\"nic\":%u",  t->nic);                else APP(",\"nic\":null");
    if (t->nacp_valid)     APP(",\"nacp\":%u", t->nacp);               else APP(",\"nacp\":null");
    if (t->ss_valid)       APP(",\"ss\":%u",   t->ss);                 else APP(",\"ss\":null");

    APP(",\"age\":%.1f}", (double)(now_ms - t->last_seen_ms) / 1000.0);
    #undef APP
    return (int)u;
}

// ---- GET / ------------------------------------------------------------------

static esp_err_t root_get(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    // -1: don't count the NUL EMBED_TXTFILES appends.
    return httpd_resp_send(req, (const char *)_binary_index_html_start,
                           _binary_index_html_end - _binary_index_html_start - 1);
}

// ---- GET /getStatus ---------------------------------------------------------

static esp_err_t status_get(httpd_req_t *req)
{
    net_sta_status_t sta;
    net_sta_status(&sta);

    char buf[448];
    int n = snprintf(buf, sizeof buf,
        "{\"version\":\"%s\",\"pong\":%d,\"degraded\":%d,\"utc_ok\":%d,"
        "\"es\":%u,\"es_rej\":%u,\"uat\":%u,\"uat_rej\":%u,\"uplink\":%u,"
        "\"pong_err\":%u,\"clients\":%d,\"leases\":%d,"
        "\"sta_en\":%d,\"sta_up\":%d,\"sta_ip\":\"%s\",\"sta_gw\":\"%s\","
        "\"sta_dns\":\"%s\"}",
        STRATUX_ESP32_VERSION, g_status.pong_connected, g_status.pong_degraded,
        g_status.utc_ok,
        (unsigned)g_status.es_msgs, (unsigned)g_status.es_rejected,
        (unsigned)g_status.uat_msgs, (unsigned)g_status.uat_rejected,
        (unsigned)g_status.uat_uplink_msgs,
        (unsigned)g_status.pong_errors, net_client_count(), net_lease_count(),
        sta.enabled, sta.connected, sta.ip, sta.gw, sta.dns);
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, buf, n);
}

// ---- GET /getSettings / POST /setSettings ------------------------------------

// Built with cJSON so SSID/password/region values containing quotes or
// backslashes are escaped instead of corrupting the response.
static esp_err_t settings_get(httpd_req_t *req)
{
    cJSON *j = cJSON_CreateObject();
    if (!j) return ESP_FAIL;

    char own[8];
    snprintf(own, sizeof own, "%06lX", (unsigned long)g_settings.ownship_modes);
    cJSON_AddStringToObject(j, "wifi_ssid", g_settings.wifi_ssid);
    cJSON_AddStringToObject(j, "wifi_pass", g_settings.wifi_pass);
    cJSON_AddNumberToObject(j, "wifi_chan", g_settings.wifi_chan);
    cJSON_AddBoolToObject(j, "pong_en", g_settings.pong_en);
    cJSON_AddBoolToObject(j, "es_en", g_settings.es_en);
    cJSON_AddBoolToObject(j, "uat_en", g_settings.uat_en);
    cJSON_AddStringToObject(j, "ownship", own);
    cJSON_AddNumberToObject(j, "alt_off", g_settings.alt_off);
    cJSON_AddStringToObject(j, "region", g_settings.region);
    cJSON_AddBoolToObject(j, "sta_en", g_settings.sta_en);
    cJSON_AddStringToObject(j, "sta_ssid", g_settings.sta_ssid);
    cJSON_AddStringToObject(j, "sta_pass", g_settings.sta_pass);
    cJSON_AddStringToObject(j, "gdl90_dest", g_settings.gdl90_dest);

    char *out = cJSON_PrintUnformatted(j);
    cJSON_Delete(j);
    if (!out) return ESP_FAIL;
    httpd_resp_set_type(req, "application/json");
    esp_err_t err = httpd_resp_sendstr(req, out);
    cJSON_free(out);
    return err;
}

static void reboot_timer_cb(void *arg)
{
    (void)arg;
    esp_restart();
}

// Schedule a restart far enough out for the HTTP response to flush.
static void schedule_reboot(void)
{
    static esp_timer_handle_t t;
    const esp_timer_create_args_t args = {
        .callback = reboot_timer_cb,
        .name = "setcfg_reboot",
    };
    if (!t && esp_timer_create(&args, &t) != ESP_OK) {
        esp_restart();   // can't even make a timer; restart now
        return;
    }
    esp_timer_start_once(t, 750 * 1000);
}

// Only keys present in the body are applied (PATCH semantics, like Stratux).
// WiFi-affecting changes persist and then reboot the device — the SoftAP can't
// re-key live without dropping every client anyway.
static esp_err_t settings_post(httpd_req_t *req)
{
    char body[512];
    int total = req->content_len;
    if (total <= 0 || total >= (int)sizeof(body)) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "bad length");
        return ESP_FAIL;
    }
    int got = 0;
    while (got < total) {
        int r = httpd_req_recv(req, body + got, total - got);
        if (r <= 0) return ESP_FAIL;
        got += r;
    }
    body[total] = '\0';

    cJSON *j = cJSON_Parse(body);
    if (!j) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "bad JSON");
        return ESP_FAIL;
    }

    // Stage into a local copy so a validation failure mid-body (goto bad)
    // leaves the live g_settings untouched — no partially-applied request.
    settings_t next = g_settings;
    const cJSON *v;

    v = cJSON_GetObjectItem(j, "wifi_ssid");
    if (cJSON_IsString(v)) {
        size_t l = strlen(v->valuestring);
        if (l < 1 || l > 32) goto bad;
        strcpy(next.wifi_ssid, v->valuestring);
    }
    v = cJSON_GetObjectItem(j, "wifi_pass");
    if (cJSON_IsString(v)) {
        size_t l = strlen(v->valuestring);
        if (l != 0 && (l < 8 || l > 64)) goto bad;   // WPA2 bounds; empty = open
        strcpy(next.wifi_pass, v->valuestring);
    }
    v = cJSON_GetObjectItem(j, "wifi_chan");
    if (cJSON_IsNumber(v)) {
        int c = v->valueint;
        if (c < 1 || c > 13) goto bad;
        next.wifi_chan = (uint8_t)c;
    }
    v = cJSON_GetObjectItem(j, "pong_en");
    if (cJSON_IsBool(v)) next.pong_en = cJSON_IsTrue(v);
    v = cJSON_GetObjectItem(j, "es_en");
    if (cJSON_IsBool(v)) next.es_en = cJSON_IsTrue(v);
    v = cJSON_GetObjectItem(j, "uat_en");
    if (cJSON_IsBool(v)) next.uat_en = cJSON_IsTrue(v);
    v = cJSON_GetObjectItem(j, "ownship");
    if (cJSON_IsString(v)) {
        // 1-6 hex digits, fully consumed — reject "XYZ" or a 7+ digit address
        // instead of silently masking it to 24 bits.
        char *end;
        unsigned long o = strtoul(v->valuestring, &end, 16);
        if (end == v->valuestring || *end != '\0' || o > 0xFFFFFF) goto bad;
        next.ownship_modes = (uint32_t)o;
    }
    v = cJSON_GetObjectItem(j, "alt_off");
    if (cJSON_IsNumber(v)) next.alt_off = v->valueint;
    v = cJSON_GetObjectItem(j, "region");
    if (cJSON_IsString(v)) {
        if (strlen(v->valuestring) > 3) goto bad;
        strcpy(next.region, v->valuestring);
    }
    v = cJSON_GetObjectItem(j, "sta_en");
    if (cJSON_IsBool(v)) next.sta_en = cJSON_IsTrue(v);
    v = cJSON_GetObjectItem(j, "sta_ssid");
    if (cJSON_IsString(v)) {
        if (strlen(v->valuestring) > 32) goto bad;   // empty ok while disabled
        strcpy(next.sta_ssid, v->valuestring);
    }
    v = cJSON_GetObjectItem(j, "sta_pass");
    if (cJSON_IsString(v)) {
        size_t l = strlen(v->valuestring);
        if (l != 0 && (l < 8 || l > 64)) goto bad;   // WPA2 bounds; empty = open
        strcpy(next.sta_pass, v->valuestring);
    }
    if (next.sta_en && !next.sta_ssid[0]) goto bad;  // client mode needs an SSID
    v = cJSON_GetObjectItem(j, "gdl90_dest");
    if (cJSON_IsString(v)) {
        if (strlen(v->valuestring) >= sizeof(next.gdl90_dest)) goto bad;
        strcpy(next.gdl90_dest, v->valuestring);
        // Last staging step: validates the list AND applies it live (no reboot
        // needed for target changes). The only failure after this is the NVS
        // save, whose revert path below restores the previous list.
        if (net_set_static_dest(next.gdl90_dest) < 0) goto bad;
    }

    cJSON_Delete(j);

    bool wifi_changed =
        strcmp(next.wifi_ssid, g_settings.wifi_ssid) != 0 ||
        strcmp(next.wifi_pass, g_settings.wifi_pass) != 0 ||
        next.wifi_chan != g_settings.wifi_chan ||
        next.sta_en != g_settings.sta_en ||
        strcmp(next.sta_ssid, g_settings.sta_ssid) != 0 ||
        strcmp(next.sta_pass, g_settings.sta_pass) != 0;

    settings_t prev = g_settings;
    g_settings = next;
    esp_err_t err = settings_save();
    if (err != ESP_OK) {
        g_settings = prev;   // keep RAM consistent with what NVS still holds
        net_set_static_dest(prev.gdl90_dest);
        ESP_LOGE(TAG, "settings_save failed: %s", esp_err_to_name(err));
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "NVS write failed");
        return ESP_FAIL;
    }

    httpd_resp_set_type(req, "application/json");
    if (wifi_changed) {
        ESP_LOGW(TAG, "WiFi settings changed; rebooting to apply");
        httpd_resp_sendstr(req, "{\"ok\":true,\"reboot\":true}");
        schedule_reboot();
    } else {
        httpd_resp_sendstr(req, "{\"ok\":true}");
    }
    return ESP_OK;

bad:
    cJSON_Delete(j);
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "invalid value");
    return ESP_FAIL;
}

// ---- GET /getTraffic ---------------------------------------------------------

static esp_err_t traffic_get(httpd_req_t *req)
{
    size_t total = traffic_count();
    size_t n = traffic_snapshot(s_snap, WEB_TRAFFIC_ROWS);
    if (total < n) total = n;   // separate lock takes; entries may age between
    int64_t now_ms = esp_timer_get_time() / 1000;

    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr_chunk(req, "{\"traffic\":[");
    size_t emitted = 0;
    for (size_t i = 0; i < n; i++) {
        if (traffic_row_json(s_row, sizeof(s_row), &s_snap[i], now_ms) < 0)
            continue;
        if (emitted++) httpd_resp_sendstr_chunk(req, ",");
        httpd_resp_sendstr_chunk(req, s_row);
    }
    // Rows not shown: table entries beyond the snapshot cap + formatting drops.
    char tail[40];
    snprintf(tail, sizeof tail, "],\"truncated\":%u}",
             (unsigned)(total - emitted));
    httpd_resp_sendstr_chunk(req, tail);
    httpd_resp_sendstr_chunk(req, NULL);
    return ESP_OK;
}

// ---- GET /getUat --------------------------------------------------------------

static esp_err_t uat_get(httpd_req_t *req)
{
    static uat_uplink_stats_t st;   // httpd context only
    uat_uplink_get_stats(&st);
    int64_t now_ms = esp_timer_get_time() / 1000;

    char buf[256];
    int n = snprintf(buf, sizeof buf,
        "{\"uplink_frames\":%u,\"no_app_data\":%u,\"bad\":%u,"
        "\"info_frames\":%u,\"tisb\":%u,\"overflow\":%u,\"products\":[",
        (unsigned)st.frames, (unsigned)st.no_app_data, (unsigned)st.bad_frames,
        (unsigned)st.info_frames, (unsigned)st.tisb_frames, (unsigned)st.overflow);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send_chunk(req, buf, n);

    bool first = true;
    for (int i = 0; i < UAT_UPLINK_PRODUCT_SLOTS; i++) {
        const fisb_product_stat_t *p = &st.products[i];
        if (!p->count) continue;
        n = snprintf(buf, sizeof buf,
            "%s{\"id\":%u,\"name\":\"%s\",\"count\":%u,\"age\":%.0f}",
            first ? "" : ",", p->product_id, fisb_product_name(p->product_id),
            (unsigned)p->count, (double)(now_ms - p->last_ms) / 1000.0);
        httpd_resp_send_chunk(req, buf, n);
        first = false;
    }
    httpd_resp_sendstr_chunk(req, "]}");
    httpd_resp_sendstr_chunk(req, NULL);
    return ESP_OK;
}

// ---- GET /getPongLog -----------------------------------------------------------

static esp_err_t ponglog_get(httpd_req_t *req)
{
    size_t n = pong_diag_copy(s_ponglog, sizeof(s_ponglog));
    httpd_resp_set_type(req, "text/plain");
    return httpd_resp_send(req, s_ponglog, n);
}

// ---- WS /traffic ----------------------------------------------------------------

// Handshake-only handler: the UI never sends application frames; anything that
// does arrive (e.g. a close) is drained so the socket stays in sync.
static esp_err_t traffic_ws(httpd_req_t *req)
{
    if (req->method == HTTP_GET)
        return ESP_OK;   // handshake done by httpd

    httpd_ws_frame_t frame = { 0 };
    esp_err_t err = httpd_ws_recv_frame(req, &frame, 0);   // len query
    if (err != ESP_OK)
        return err;
    if (frame.len == 0)
        return ESP_OK;     // control frame with no payload — nothing to drain
    uint8_t drain[126];
    if (frame.len > sizeof(drain))
        return ESP_FAIL;   // UI never sends big frames; drop the socket
    frame.payload = drain;
    return httpd_ws_recv_frame(req, &frame, frame.len);
}

// Runs in the httpd context (queued by the 1 Hz timer): build one JSON frame
// from the traffic snapshot and fan it out to every connected WS client.
static void ws_push_work(void *arg)
{
    (void)arg;

    size_t fdn = CONFIG_LWIP_MAX_SOCKETS;
    int fds[CONFIG_LWIP_MAX_SOCKETS];
    if (httpd_get_client_list(s_server, &fdn, fds) != ESP_OK || fdn == 0)
        return;

    bool any_ws = false;
    for (size_t i = 0; i < fdn; i++) {
        if (httpd_ws_get_fd_info(s_server, fds[i]) == HTTPD_WS_CLIENT_WEBSOCKET) {
            any_ws = true;
            break;
        }
    }
    if (!any_ws)
        return;   // don't bother snapshotting for nobody

    size_t total = traffic_count();
    size_t n = traffic_snapshot(s_snap, WEB_TRAFFIC_ROWS);
    if (total < n) total = n;   // separate lock takes; entries may age between
    int64_t now_ms = esp_timer_get_time() / 1000;

    size_t u = (size_t)snprintf(s_wsbuf, sizeof(s_wsbuf), "{\"traffic\":[");
    // Count everything not shown, starting with entries beyond the snapshot cap.
    size_t dropped = total - n, emitted = 0;
    for (size_t i = 0; i < n; i++) {
        int w = traffic_row_json(s_row, sizeof(s_row), &s_snap[i], now_ms);
        if (w < 0) { dropped++; continue; }
        // +24 leaves room for the closing "],"truncated":NNN}" tail.
        if (u + (size_t)w + 24 >= sizeof(s_wsbuf)) {
            dropped += n - i;
            break;
        }
        if (emitted) s_wsbuf[u++] = ',';
        memcpy(&s_wsbuf[u], s_row, (size_t)w);
        u += (size_t)w;
        emitted++;
    }
    u += (size_t)snprintf(&s_wsbuf[u], sizeof(s_wsbuf) - u,
                          "],\"truncated\":%u}", (unsigned)dropped);

    httpd_ws_frame_t frame = {
        .type = HTTPD_WS_TYPE_TEXT,
        .payload = (uint8_t *)s_wsbuf,
        .len = u,
    };
    for (size_t i = 0; i < fdn; i++) {
        if (httpd_ws_get_fd_info(s_server, fds[i]) != HTTPD_WS_CLIENT_WEBSOCKET)
            continue;
        // A failed push means the peer is gone or wedged (e.g. deauthed mid-
        // session): close the session, or the dead socket pins one of httpd's
        // few slots forever (LRU purge skips it — our 1 Hz sends keep it
        // "active") and starves new connections.
        if (httpd_ws_send_frame_async(s_server, fds[i], &frame) != ESP_OK) {
            ESP_LOGW(TAG, "WS push to fd %d failed; closing dead session", fds[i]);
            httpd_sess_trigger_close(s_server, fds[i]);
        }
    }
}

static void ws_timer_cb(void *arg)
{
    (void)arg;
    if (s_server)
        httpd_queue_work(s_server, ws_push_work, NULL);
}

// ---- startup --------------------------------------------------------------------

void web_start(void)
{
    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.max_uri_handlers = 12;
    cfg.lru_purge_enable = true;   // EFB + browser + WS: recycle idle sockets

    if (httpd_start(&s_server, &cfg) != ESP_OK) {
        ESP_LOGE(TAG, "httpd_start failed");
        s_server = NULL;
        return;
    }

    const httpd_uri_t uris[] = {
        { .uri = "/",            .method = HTTP_GET,  .handler = root_get },
        { .uri = "/getStatus",   .method = HTTP_GET,  .handler = status_get },
        { .uri = "/getSettings", .method = HTTP_GET,  .handler = settings_get },
        { .uri = "/setSettings", .method = HTTP_POST, .handler = settings_post },
        { .uri = "/getTraffic",  .method = HTTP_GET,  .handler = traffic_get },
        { .uri = "/getUat",      .method = HTTP_GET,  .handler = uat_get },
        { .uri = "/getPongLog",  .method = HTTP_GET,  .handler = ponglog_get },
        { .uri = "/traffic",     .method = HTTP_GET,  .handler = traffic_ws,
          .is_websocket = true },
    };
    for (size_t i = 0; i < sizeof(uris) / sizeof(uris[0]); i++)
        httpd_register_uri_handler(s_server, &uris[i]);

    // 1 Hz live-traffic push (matches the traffic_mgr/GDL90 cadence).
    static esp_timer_handle_t t;
    const esp_timer_create_args_t targs = {
        .callback = ws_timer_cb,
        .name = "ws_traffic",
    };
    if (esp_timer_create(&targs, &t) == ESP_OK)
        esp_timer_start_periodic(t, 1000 * 1000);

    ESP_LOGI(TAG, "web UI up (http://192.168.10.1/)");
}
