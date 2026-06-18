#pragma once
#include "sdkconfig.h"
#include "driver/gpio.h"
#include "driver/uart.h"             // UART_NUM_2

#if CONFIG_BOARD_TTGO_T8

// TTGO T8 (LilyGO) pin map. ESP32 GPIO is heavily multiplexed and the T8
// reserves pins for SD, PSRAM, and flash; this map is deliberately
// conflict-free for the Pong UART.
#define BOARD_NAME       "TTGO T8"

// --- Pong radio — UART2 @ 3 Mbaud, 8N1, static RTS ---
#define PONG_UART_PORT   UART_NUM_2
#define PONG_UART_BAUD   3000000
#define PONG_RX_GPIO     GPIO_NUM_35   // input-only; NO internal pull — Pong must drive idle-high (or add external pull-up)
#define PONG_TX_GPIO     GPIO_NUM_33   // RTC GPIO, free output
#define PONG_RTS_GPIO    GPIO_NUM_32   // RTC GPIO; drive as STATIC level to ClearRTS() level, NOT HW flow control
#define PONG_RTS_ENABLED 1

// --- Console — UART0 (USB-serial bridge); leave alone ---
//   TX=GPIO1  RX=GPIO3

// --- Reserved by hardware — do NOT reuse ---
//   GPIO16/17 = PSRAM data (Bug B: uart_set_pin silently strands a UART here)
//   GPIO6-11  = internal flash
//   GPIO14/15/2 (+13) = microSD SDMMC 1-bit, if the slot is populated
//   strapping pins: 0, 2, 5, 12, 15  (avoid for new outputs; GPIO4 is free)

// --- Future milestones ---
#define I2C_SDA_GPIO     GPIO_NUM_21   // M3/M4 (GPS aux / IMU / OLED)
#define I2C_SCL_GPIO     GPIO_NUM_22
#define GPS_RX_GPIO      GPIO_NUM_34   // M3 — input-only (RX)
#define GPS_TX_GPIO      GPIO_NUM_4

#elif CONFIG_BOARD_ESP32_DEVKITC_V4

// ESP32-DevKitC V4 pin map. External SPI SD breakout is wired to VSPI:
//   MISO=GPIO19, MOSI=GPIO23, CS=GPIO5, CLK=GPIO18
// UART0 GPIO1/3 remains reserved for programming/logging.
#define BOARD_NAME       "ESP32-DevKitC V4"

// --- Pong radio — UART2 @ 3 Mbaud, 8N1, no RTS wire ---
#define PONG_UART_PORT   UART_NUM_2
#define PONG_UART_BAUD   3000000
#define PONG_RX_GPIO     GPIO_NUM_34   // input-only; Pong must drive idle-high (or add external pull-up)
#define PONG_TX_GPIO     GPIO_NUM_25
#define PONG_RTS_GPIO    GPIO_NUM_NC
#define PONG_RTS_ENABLED 0

// --- Console — UART0 (USB-serial bridge); leave alone for programming ---
//   TX=GPIO1  RX=GPIO3

// --- SPI microSD breakout on VSPI ---
#define SD_SPI_MISO_GPIO GPIO_NUM_19
#define SD_SPI_MOSI_GPIO GPIO_NUM_23
#define SD_SPI_CS_GPIO   GPIO_NUM_5
#define SD_SPI_CLK_GPIO  GPIO_NUM_18

// --- Future milestones ---
#define I2C_SDA_GPIO     GPIO_NUM_21
#define I2C_SCL_GPIO     GPIO_NUM_22
#define GPS_RX_GPIO      GPIO_NUM_NC
#define GPS_TX_GPIO      GPIO_NUM_NC

#else
#error "No supported board selected"
#endif
