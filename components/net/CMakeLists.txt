idf_component_register(
    SRCS "src/net.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_wifi esp_netif esp_event lwip common
)
