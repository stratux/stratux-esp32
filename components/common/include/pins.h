#pragma once
#include "driver/gpio.h"
#include "driver/uart.h"             // UART_NUM_2

// TTGO T8 (LilyGO) pin map. ESP32 GPIO is heavily multiplexed and the T8
// reserves pins for SD, PSRAM, and flash; this map is deliberately
// conflict-free for the Pong UART.

// --- Pong radio — UART2 @ 3 Mbaud, 8N1, static RTS ---
#define PONG_UART_PORT   UART_NUM_2
#define PONG_UART_BAUD   3000000
#define PONG_RX_GPIO     GPIO_NUM_35   // input-only; NO internal pull — Pong must drive idle-high (or add external pull-up)
#define PONG_TX_GPIO     GPIO_NUM_33   // RTC GPIO, free output
#define PONG_RTS_GPIO    GPIO_NUM_32   // RTC GPIO; drive as STATIC level to ClearRTS() level, NOT HW flow control

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
