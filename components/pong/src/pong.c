#include "pong.h"
#include <string.h>
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "pins.h"
#include "settings.h"
#include "stratux_status.h"
#include "modes.h"
#include "uat.h"

static const char *TAG = "pong";

#define PONG_RX_BUF   (4 * 1024)   // 3 Mbaud is bursty; give the driver headroom
#define PONG_LINE_MAX 256

// Classify a line by its first character (Appendix A).
static pong_line_kind_t classify(char c)
{
    switch (c) {
        case '*':  return PONG_LINE_1090ES;
        case '-':  return PONG_LINE_UAT_DOWN;
        case '+':  return PONG_LINE_UAT_UP;
        case '.':  return PONG_LINE_HEARTBEAT;
        case '\'': return PONG_LINE_STATUS;
        default:   return PONG_LINE_LOG;
    }
}

// Parse a complete line into a frame and dispatch it to the right decoder.
static void handle_line(char *line)
{
    pong_frame_t f = { .kind = classify(line[0]) };

    switch (f.kind) {
        case PONG_LINE_HEARTBEAT:
            g_status.pong_connected = true;
            return;

        case PONG_LINE_LOG:
            if (strstr(line, "ERROR SPI")) {
                g_status.pong_errors++;
                ESP_LOGW(TAG, "Pong reported ERROR SPI — TODO(M1): real recovery "
                              "(reopen UART / toggle reset GPIO / mark degraded)");
            }
            return;

        case PONG_LINE_1090ES:
            // TODO(M1): copy hex (between '*' and ';') into f.hex/f.hex_len,
            //   parse trailing "ss=" as HEX. Then:
            if (g_settings.es_en) modes_decode_frame(&f);
            return;

        case PONG_LINE_UAT_DOWN:
        case PONG_LINE_UAT_UP:
            // TODO(M1): copy hex, parse "rs="/"ss=" (ss is hex int8; 0x80 = errored).
            if (g_settings.uat_en) uat_decode_frame(&f);
            return;

        default:
            return;
    }
}

void pong_rx_task(void *arg)
{
    (void)arg;

    // UART2 @ 3 Mbaud, 8N1. Stratux clears RTS once and never toggles it, with
    // no CTS — so we do NOT use UART_HW_FLOWCTRL_RTS. Drive GPIO32 as a static
    // level instead. Only switch to hardware RTS if bench
    // testing proves the Pong actually pauses/resumes on it. Keep RTS/TX off
    // GPIO16/17 (PSRAM data, Bug B).
    uart_config_t cfg = {
        .baud_rate  = PONG_UART_BAUD,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(PONG_UART_PORT, PONG_RX_BUF, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(PONG_UART_PORT, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(PONG_UART_PORT, PONG_TX_GPIO, PONG_RX_GPIO,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    // RTS as a static GPIO level (ClearRTS equivalent). TODO(bring-up): confirm
    // the polarity the Pong expects on real hardware.
    gpio_set_direction(PONG_RTS_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(PONG_RTS_GPIO, 0);

    ESP_LOGI(TAG, "Pong UART%d up @ %d baud (RX=%d TX=%d RTS=%d)",
             PONG_UART_PORT, PONG_UART_BAUD, PONG_RX_GPIO, PONG_TX_GPIO, PONG_RTS_GPIO);

    static char line[PONG_LINE_MAX];
    size_t len = 0;
    uint8_t byte;

    for (;;) {
        // TODO(M1): batch reads (uart_read_bytes into a chunk) for 3 Mbaud
        // throughput; byte-at-a-time here is just the readable skeleton form.
        int n = uart_read_bytes(PONG_UART_PORT, &byte, 1, portMAX_DELAY);
        if (n != 1) continue;

        if (byte == '\n' || byte == '\r') {
            if (len == 0) continue;
            line[len] = '\0';
            handle_line(line);
            len = 0;
        } else if (len < PONG_LINE_MAX - 1) {
            line[len++] = (char)byte;
        } else {
            len = 0;   // overlong line; resync on next newline
        }
    }
}
