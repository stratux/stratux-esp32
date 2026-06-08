# Placeholder for the M5 tiered weather store (PSRAM -> himem -> microSD), to be
# ported from connext-emulator/esp32/components/wxstore. M5
# also requires the WROVER/8 MB-PSRAM overlay. Registers an empty component so
# the build stays warning-free until then; not yet in main's REQUIRES.
idf_component_register()
