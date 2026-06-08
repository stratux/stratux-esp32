idf_component_register(
    SRCS "src/gdl90_out.c"
    INCLUDE_DIRS "include"
    # net: net_gdl90_send()   traffic: traffic_snapshot()   common: g_status
    REQUIRES net traffic common
)
