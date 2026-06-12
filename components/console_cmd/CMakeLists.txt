idf_component_register(
    SRCS "src/console_cmd.c"
    INCLUDE_DIRS "include"
    # common: settings.h (NVS-backed g_settings); net: STA status + static dests
    REQUIRES common net
    # driver: UART0 reader task (production builds only)
    PRIV_REQUIRES driver
)
