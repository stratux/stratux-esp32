idf_component_register(
    SRCS "src/uat.c"
         "src/uat_decode.c"
    INCLUDE_DIRS "include"
    # uat_decode.h is private.
    PRIV_INCLUDE_DIRS "src"
    # common: pong_frame.h, stratux_status.h   traffic: traffic_upsert()
    REQUIRES common traffic
)
