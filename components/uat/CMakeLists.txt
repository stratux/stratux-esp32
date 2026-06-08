idf_component_register(
    SRCS "src/uat.c"
    INCLUDE_DIRS "include"
    # common: pong_frame.h, stratux_status.h   traffic: traffic_upsert()
    REQUIRES common traffic
)
