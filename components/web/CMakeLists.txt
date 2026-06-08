idf_component_register(
    SRCS "src/web.c"
    INCLUDE_DIRS "include"
    # esp_http_server: server + WS   net: client count   common: g_status
    # traffic added at M2 for the live traffic feed.
    REQUIRES esp_http_server net common
)
