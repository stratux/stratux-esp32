#include "pong.h"
#include <string.h>
#include "sdkconfig.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "pins.h"
#include "settings.h"
#include "stratux_status.h"
#include "modes.h"
#include "uat.h"

static const char *TAG = "pong";

// Pong frame source (see components/pong/Kconfig). Default is the real radio on
// UART2 @ 3 Mbaud. CONFIG_PONG_SOURCE_CONSOLE instead reads Pong-format lines
// from the console UART0 (onboard USB bridge) so a host can inject a recorded
// capture with tools/pong_replay.py over the single USB cable — the bridge is
// unreliable above ~115200 (AGENTS.md), so that path runs at a lower baud.
#if CONFIG_PONG_SOURCE_CONSOLE
#define PONG_ACTIVE_PORT  UART_NUM_0
#define PONG_ACTIVE_BAUD  CONFIG_PONG_CONSOLE_BAUD
#else
#define PONG_ACTIVE_PORT  PONG_UART_PORT
#define PONG_ACTIVE_BAUD  PONG_UART_BAUD
#endif

#define PONG_RX_BUF   (4 * 1024)   // 3 Mbaud is bursty; give the driver headroom
// Must hold the longest Pong line. UAT uplink (FIS-B) frames reach 877 chars in
// real captures (864 hex + '+' + ";rs=..;ss=..;"); 256 silently dropped ~64% of
// uplink lines via the overlong-line resync below. PONG_HEX_MAX (896) + the
// classifier/ss/rs suffix fits comfortably in 1024.
#define PONG_LINE_MAX 1024

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
            // TODO(M1): copy hex (between '*' and the first ';') into
            //   f.hex/f.hex_len. The "ss=" field is OPTIONAL on 1090ES lines:
            //   only set f.ss/f.ss_valid if a literal "ss=" is found (HEX).
            //   Do NOT treat the trailing ";<n>" some firmware emits as ss —
            //   that is a message counter. Like Stratux, key off report[0].
            if (g_settings.es_en) modes_decode_frame(&f);
            return;

        case PONG_LINE_UAT_DOWN:
        case PONG_LINE_UAT_UP:
            // TODO(M1): copy hex, parse "rs="/"ss=" (ss is hex int8; 0x80 =
            //   errored). UAT lines carry ss= reliably; set f.ss_valid when found.
            if (g_settings.uat_en) uat_decode_frame(&f);
            return;

        default:
            return;
    }
}

void pong_rx_task(void *arg)
{
    (void)arg;

    // 8N1, no HW flow control (Stratux clears RTS once and never toggles it,
    // with no CTS). Baud/port depend on the source: real radio = UART2 @ 3
    // Mbaud; console replay = UART0 @ PONG_CONSOLE_BAUD.
    uart_config_t cfg = {
        .baud_rate  = PONG_ACTIVE_BAUD,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    // The console driver may already own UART0 for logging; only install if not.
    if (!uart_is_driver_installed(PONG_ACTIVE_PORT)) {
        ESP_ERROR_CHECK(uart_driver_install(PONG_ACTIVE_PORT, PONG_RX_BUF, 0, 0, NULL, 0));
    }
    ESP_ERROR_CHECK(uart_param_config(PONG_ACTIVE_PORT, &cfg));

#if !CONFIG_PONG_SOURCE_CONSOLE
    // Real radio: route UART2 to the Pong pins and hold RTS at the static
    // ClearRTS level on GPIO32. Keep RTS/TX off GPIO16/17 (PSRAM data, Bug B).
    // Only switch to hardware RTS if bench testing proves the Pong actually
    // pauses/resumes on it. TODO(bring-up): confirm the RTS polarity on real HW.
    ESP_ERROR_CHECK(uart_set_pin(PONG_UART_PORT, PONG_TX_GPIO, PONG_RX_GPIO,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    gpio_set_direction(PONG_RTS_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(PONG_RTS_GPIO, 0);
    ESP_LOGI(TAG, "Pong source: radio on UART%d @ %d baud (RX=%d TX=%d RTS=%d)",
             PONG_ACTIVE_PORT, PONG_ACTIVE_BAUD, PONG_RX_GPIO, PONG_TX_GPIO, PONG_RTS_GPIO);
#else
    // Console replay: UART0 keeps its existing GPIO1/3 console pins; do NOT call
    // uart_set_pin or touch the RTS GPIO. Logs still flow out on UART0 TX.
    ESP_LOGW(TAG, "Pong source: CONSOLE replay on UART%d @ %d baud (dev harness, "
                  "not the real radio)", PONG_ACTIVE_PORT, PONG_ACTIVE_BAUD);
#endif

    static char line[PONG_LINE_MAX];
    size_t len = 0;
    uint8_t byte;

    for (;;) {
        // TODO(M1): batch reads (uart_read_bytes into a chunk) for 3 Mbaud
        // throughput; byte-at-a-time here is just the readable skeleton form.
        int n = uart_read_bytes(PONG_ACTIVE_PORT, &byte, 1, portMAX_DELAY);
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
