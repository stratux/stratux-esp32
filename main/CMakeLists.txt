idf_component_register(
    SRCS
        "main.c"
    INCLUDE_DIRS "."
    REQUIRES
        nvs_flash
        esp_event
        common
        pong
        modes
        uat
        traffic
        gdl90_out
        net
        web
)
