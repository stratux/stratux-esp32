# GPIO Board Mapping Worksheet

Use this table to compare the existing TTGO T8 pin map with the ESP32-CAM board
pin map before adding board selection support. ESP32-CAM entries are populated
from `ESP32_CAM_V1.6.pdf` where the schematic is explicit.

Assumption for ESP32-CAM support: PSRAM, the microSD slot, and UART0 programming
must remain usable. Camera, onboard LED, flash LED, and runtime console logging
may be sacrificed. Pong wiring may be soldered directly to module/camera-related
nets instead of only using stock `P1`/`P2` headers.

| Function | Firmware macro / interface | TTGO T8 current mapping | ESP32-CAM mapping | Notes |
|---|---|---:|---:|---|
| Pong UART port | `PONG_UART_PORT` | `UART_NUM_2` | TBD; likely `UART_NUM_1` or `UART_NUM_2` remapped to direct-solder GPIOs | ESP32 UART peripherals can be routed through the GPIO matrix; choose pins from the candidate direct-solder nets below |
| Pong baud | `PONG_UART_BAUD` | `3000000` | `3000000` | Pong serial link is 3 Mbaud, 8N1 |
| Pong RX, Pong to ESP32 | `PONG_RX_GPIO` | `GPIO35` | TBD; recommended candidate `GPIO34`/`CSI_D6`, `GPIO35`/`CSI_D7`, `GPIO36`/`CSI_D4`, or `GPIO39`/`CSI_D5` | Input-only camera data pins are good RX candidates if camera is not used; they have no internal pulls |
| Pong TX, ESP32 to Pong | `PONG_TX_GPIO` | `GPIO33` | TBD; recommended candidate `GPIO18`, `GPIO19`, `GPIO21`, `GPIO22`, `GPIO23`, `GPIO25`, `GPIO26`, `GPIO27`, `GPIO32`, or `GPIO33` | Use an output-capable camera/LED/power-control net; verify solder accessibility |
| Pong RTS static level | `PONG_RTS_GPIO` | `GPIO32` | TBD; candidate from output-capable list, or leave unwired/disabled if Pong permits | Needs a safe output GPIO if Pong RTS is wired |
| Console UART0 TX | UART0 default | `GPIO1` | `GPIO1` / `U0TXD` on `P1-2`; reserve for programming | Do not assign to Pong in the normal ESP32-CAM board map |
| Console UART0 RX | UART0 default | `GPIO3` | `GPIO3` / `U0RXD` on `P1-3`; reserve for programming | Do not assign to Pong in the normal ESP32-CAM board map |
| I2C SDA | `I2C_SDA_GPIO` | `GPIO21` | `GPIO26` / `TWI_SDA` | Camera SCCB/I2C data, pulled up to 3V3 through `R18` |
| I2C SCL | `I2C_SCL_GPIO` | `GPIO22` | `GPIO27` / `TWI_SCK` | Camera SCCB/I2C clock, pulled up to 3V3 through `R17` |
| GPS RX | `GPS_RX_GPIO` | `GPIO34` | Not assigned | No dedicated GPS UART net shown; leave GPS unsupported on ESP32-CAM unless extra solder points are used |
| GPS TX | `GPS_TX_GPIO` | `GPIO4` | Not assigned | `GPIO4` is SD DATA1 / flash LED and must stay with SD |

## Board Constraints To Check

| Constraint | TTGO T8 notes | ESP32-CAM notes |
|---|---|---|
| PSRAM pins | `GPIO16`, `GPIO17` reserved | `GPIO16` = PSRAM `CS#`; `GPIO17` = `PSRAM_CLK`; also uses `SD0`, `SD1`, `SD2`, `SD3` |
| Internal flash pins | `GPIO6`-`GPIO11` reserved | `GPIO6`-`GPIO11` reserved for module flash / SDIO signals |
| microSD pins | `GPIO14`, `GPIO15`, `GPIO2`, plus `GPIO13` reserved if SD slot populated | `GPIO12`=`HS2_DATA2`, `GPIO13`=`HS2_DATA3`, `GPIO15`=`HS2_CMD`, `GPIO14`=`HS2_CLK`, `GPIO2`=`HS2_DATA0`, `GPIO4`=`HS2_DATA1` |
| Boot strapping pins | Avoid `GPIO0`, `GPIO2`, `GPIO5`, `GPIO12`, `GPIO15` for new outputs | Board uses `GPIO0` for camera MCLK/header, `GPIO2`/`GPIO12`/`GPIO15` for SD, and `GPIO5` for camera data |
| Input-only pins | `GPIO34`-`GPIO39`; no internal pulls | `GPIO34`=`CSI_D6`, `GPIO35`=`CSI_D7`, `GPIO36`=`CSI_D4`, `GPIO39`=`CSI_D5`; not free if camera is used |
| Camera pins | N/A | `GPIO0` MCLK, `GPIO5` D0, `GPIO18` D1, `GPIO19` D2, `GPIO21` D3, `GPIO36` D4, `GPIO39` D5, `GPIO34` D6, `GPIO35` D7, `GPIO22` PCLK, `GPIO23` HSYNC, `GPIO25` VSYNC, `GPIO26` SDA, `GPIO27` SCK, `GPIO32` PWR, reset via `CAM_RST` |
| Onboard LED / flash LED | N/A | `GPIO33` = onboard LED; `GPIO4` / `HS2_DATA1` drives flash LED transistor `Q1` and is also microSD DATA1 |
| USB serial / flashing pins | UART0 `GPIO1` / `GPIO3` | `GPIO1` / `U0TXD` on `P1-2`; `GPIO3` / `U0RXD` on `P1-3`; reset is `E32_RST`; `GPIO0` available on `P1-5` as `CSI_MCLK` for boot mode if pulled low; reserve these for programming |

## ESP32-CAM Pong Options With PSRAM + SD + Programming Preserved

| Option | Pong RX | Pong TX | Pong RTS | Tradeoff |
|---|---|---|---|---|
| Direct solder to camera/module nets | Any unused input-capable camera net | Any unused output-capable camera net | Any unused output-capable camera net or none | Best option; keeps UART0 for programming and preserves PSRAM + SD, but requires soldering beyond stock headers |
| Stock `P1`/`P2` headers only | No valid assignment | No valid assignment | No valid assignment | Not enough safe header GPIOs remain when PSRAM, SD, and UART0 programming are all preserved |
| Relax UART0 programming after flash | `GPIO3` / `U0RXD` | `GPIO1` / `U0TXD` | `GPIO0` or none | Possible electrically, but conflicts with the requirement to keep UART0 available for programming/debug wiring |
| Relax SD requirement | SD header pins | SD header pins | SD header pins | Possible only if microSD is not used |
| Relax PSRAM requirement | `GPIO16` / `U2RXD` | Still no clean `U2TXD` on headers | TBD | Still incomplete because only `U2RXD` is exposed |
| Use exposed `U2RXD` | Not recommended | N/A | N/A | `GPIO16` conflicts with PSRAM `CS#`, so this violates the PSRAM requirement |

With PSRAM and SD preserved, avoid these ESP32-CAM nets for Pong:

| Reserved for | GPIOs / nets |
|---|---|
| PSRAM | `GPIO16`, `GPIO17`, plus module SDIO flash/PSRAM bus signals |
| microSD | `GPIO12`, `GPIO13`, `GPIO15`, `GPIO14`, `GPIO2`, `GPIO4` |
| UART0 flashing/programming | `GPIO1`, `GPIO3`; also keep `GPIO0` controllable for bootloader entry |

Candidate non-header camera nets if the camera is not needed:

| Candidate role | GPIOs / nets | Notes |
|---|---|---|
| RX candidates | `GPIO34`/`CSI_D6`, `GPIO35`/`CSI_D7`, `GPIO36`/`CSI_D4`, `GPIO39`/`CSI_D5`, plus any output-capable candidates below | `GPIO34`-`GPIO39` are input-only and fine for UART RX, but have no internal pulls |
| TX / RTS candidates | `GPIO18`/`CSI_D1`, `GPIO19`/`CSI_D2`, `GPIO21`/`CSI_D3`, `GPIO22`/`CSI_PCLK`, `GPIO23`/`CSI_HSYNC`, `GPIO25`/`CSI_VSYNC`, `GPIO26`/`TWI_SDA`, `GPIO27`/`TWI_SCK`, `GPIO32`/`CAM_PWR`, `GPIO33`/`LED` | These require soldering to camera/module nets or related components; avoid `GPIO0`/`GPIO5` unless boot strapping behavior is verified |

## ESP32-CAM Exposed Headers From Schematic

| Header | Pin | Net | ESP32 GPIO / note |
|---|---:|---|---|
| `P1` | 1 | `GND` | Ground |
| `P1` | 2 | `U0TXD` | `GPIO1` |
| `P1` | 3 | `U0RXD` | `GPIO3` |
| `P1` | 4 | Power select node | Tied to `3V3` or `5V` through `R1`/`R2` 0-ohm options |
| `P1` | 5 | `CSI_MCLK` | `GPIO0`; boot strapping pin |
| `P1` | 6 | `GND` | Ground |
| `P1` | 7 | `U2RXD` | `GPIO16`; conflicts with PSRAM `CS#` if PSRAM is populated/enabled |
| `P1` | 8 | `3V3` | 3.3 V rail |
| `P2` | 1 | `HS2_DATA1` | `GPIO4`; also flash LED control |
| `P2` | 2 | `HS2_DATA0` | `GPIO2`; boot strapping pin |
| `P2` | 3 | `HS2_CLK` | `GPIO14` |
| `P2` | 4 | `HS2_CMD` | `GPIO15`; boot strapping pin |
| `P2` | 5 | `HS2_DATA3` | `GPIO13` |
| `P2` | 6 | `HS2_DATA2` | `GPIO12`; boot strapping pin |
| `P2` | 7 | `GND` | Ground |
| `P2` | 8 | `5V` | 5 V input rail |
