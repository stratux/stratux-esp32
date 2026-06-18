# ESP32-DevKitC V4 SD Card Wiring

Source schematic: `esp32_devkitc_v4_sch.pdf`, connector headers `J2` and `J3`.

Use the GPIO-numbered pins for an external microSD socket or breakout board. Do
not use the header pins labeled `SD0`, `SD1`, `SD2`, `SD3`, `CMD`, or `CLK`; on
this board those labels are the ESP32 module flash/SDIO bus signals.

## Recommended SPI Breakout Wiring

Use this table for a microSD breakout with pins labeled `GND`, `MISO`, `MOSI`,
`CS`, `CLK`, and `3V3`.

| SD breakout pin | ESP32 signal | DevKitC header pin | Notes |
|---|---|---|---|
| `GND` | `GND` | `J3-1` or `J2-14` | Common ground |
| `MISO` | `GPIO19` / VSPI MISO | `J3-8` | Card data out to ESP32 |
| `MOSI` | `GPIO23` / VSPI MOSI | `J3-2` | ESP32 data out to card |
| `CS` | `GPIO5` / VSPI CS | `J3-10` | Chip select; boot strapping pin, keep pulled high at reset |
| `CLK` | `GPIO18` / VSPI SCLK | `J3-9` | SPI clock |
| `3V3` | `3V3` | `J2-1` | Use 3.3 V only |

This is SPI mode, not SDMMC mode. It uses fewer pins and matches common microSD
socket boards.

## Pong Wiring

These pins are used by the `BOARD=devkitc_v4` firmware pin map:

| Pong signal | ESP32 signal | DevKitC header pin | Notes |
|---|---|---|---|
| Pong TX to ESP32 RX | `GPIO34` | `J2-5` | Input-only GPIO; Pong must drive idle-high or provide an external pull-up |
| ESP32 TX to Pong RX | `GPIO25` | `J2-9` | Output-capable GPIO |
| Pong RTS | Not wired | N/A | Firmware leaves RTS disabled for this board |

## Recommended SDMMC 1-Bit Wiring

Use this only for a bare SD socket wired for SDMMC mode, not for a breakout that
labels pins as `MISO`/`MOSI`.

| SD socket signal | ESP32 signal | DevKitC header pin | Notes |
|---|---|---|---|
| `VDD` | `3V3` | `J2-1` | Use 3.3 V for a bare socket board |
| `GND` | `GND` | `J2-14` or `J3-1` | Common ground |
| `CLK` | `GPIO14` / `HS2_CLK` | `J2-12` | SD clock |
| `CMD` | `GPIO15` / `HS2_CMD` | `J3-16` | Add/verify pull-up to 3.3 V |
| `DAT0` | `GPIO2` / `HS2_DATA0` | `J3-15` | Add/verify pull-up to 3.3 V |
| `DAT1` | `GPIO4` / `HS2_DATA1` | `J3-13` | Optional for 4-bit mode; leave unused for 1-bit |
| `DAT2` | `GPIO12` / `HS2_DATA2` | `J2-13` | Optional for 4-bit mode; leave unused for 1-bit; boot strapping caution |
| `DAT3` / `CD` | `GPIO13` / `HS2_DATA3` | `J2-15` | Optional card-detect / 4-bit data; add/verify pull-up if used |

For the current firmware pin plan, SDMMC 1-bit only needs `CLK`, `CMD`, `DAT0`,
`3V3`, and `GND`.

## Pins To Avoid For External SD

These DevKitC header labels look tempting, but they are the module flash/SDIO
bus and should not be wired to the external card socket:

| Header pin | Label |
|---|---|
| `J2-16` | `SD2` |
| `J2-17` | `SD3` |
| `J2-18` | `CMD` |
| `J3-17` | `SD1` |
| `J3-18` | `SD0` |
| `J3-19` | `CLK` |

## Pull-Ups

Most SD card sockets need pull-ups on `CMD` and data lines. If the socket board
does not already include them, add about 10 kOhm to 47 kOhm from `CMD`, `DAT0`,
and any used `DAT1`/`DAT2`/`DAT3` lines to 3.3 V. Do not pull up `CLK`.
