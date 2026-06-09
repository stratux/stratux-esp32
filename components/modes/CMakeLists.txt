idf_component_register(
    SRCS "src/modes.c"
         "src/mode_s_decode.c"
         "src/cpr.c"
         "src/cpr_store.c"
         "src/crc.c"
         "src/ais_charset.c"
    INCLUDE_DIRS "include"
    # crc.h / cpr.h / mode_s_decode.h / cpr_store.h / ais_charset.h are private.
    PRIV_INCLUDE_DIRS "src"
    # common: pong_frame.h, stratux_status.h   traffic: traffic_upsert()
    REQUIRES common traffic
    PRIV_REQUIRES esp_timer
)
