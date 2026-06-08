# Shared, low-level pieces that every other component (and main) may need:
#   pins.h           — TTGO T8 board pin map
#   settings.{c,h}   — NVS-backed config + g_settings
#   stratux_status.h — cross-cutting runtime status snapshot (g_status)
# This lives in a shared component (not main/) because ESP-IDF components
# cannot depend on `main`; the worker components include these headers.
idf_component_register(
    SRCS
        "src/stratux_status.c"
        "src/settings.c"
    INCLUDE_DIRS "include"
    REQUIRES nvs_flash
)
