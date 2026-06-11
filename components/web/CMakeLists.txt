idf_component_register(
    SRCS "src/web.c"
    INCLUDE_DIRS "include"
    # esp_http_server: server + WS   net: client/lease counts   common:
    # g_status + settings   traffic: live feed   uat: FIS-B product stats
    # pong: raw-line diag ring   json: cJSON for /setSettings   esp_timer:
    # WS push timer + ages
    REQUIRES esp_http_server net common traffic uat pong json esp_timer
    # Web UI is embedded in the app image (NUL-terminated); the FAT `storage`
    # partition route is deferred (see AGENTS.md).
    EMBED_TXTFILES "../../www/index.html"
)
