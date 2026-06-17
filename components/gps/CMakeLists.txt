idf_component_register(
    SRCS "src/gps.c"
    INCLUDE_DIRS "include"
    REQUIRES common driver esp_timer
)
