idf_component_register(
    SRCS "src/pong.c"
    INCLUDE_DIRS "include"
    # common: pins.h, settings.h, stratux_status.h, pong_frame.h
    # modes/uat: decode entry points (pong dispatches synchronously)
    REQUIRES driver common modes uat
)
