#include "pong.h"
#include <string.h>
#include <stdlib.h>
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

        case PONG_LINE_1090ES: {
            // The hex frame is everything between '*' and the first ';'
            // (e.g. *8DC01C2860C37797E9732E555B23;ss=049D;). Like Stratux, we
            // key off that first field and ignore the rest.
            const char *body = line + 1;
            const char *semi = strchr(body, ';');
            size_t hlen = semi ? (size_t)(semi - body) : strlen(body);
            if (hlen >= PONG_HEX_MAX) hlen = PONG_HEX_MAX - 1;
            memcpy(f.hex, body, hlen);
            f.hex[hlen] = '\0';
            f.hex_len = (uint16_t)hlen;

            // "ss=" is OPTIONAL on 1090ES lines and is a HEX log-detector
            // reading. Only set it if a literal "ss=" field is present; never
            // treat a trailing ";<n>" message counter as signal strength.
            const char *ss = strstr(line, "ss=");
            if (ss) {
                f.ss = (uint16_t)strtoul(ss + 3, NULL, 16);
                f.ss_valid = 1;
            }

            if (g_settings.es_en) modes_decode_frame(&f);
            return;
        }

        case PONG_LINE_UAT_DOWN:
        case PONG_LINE_UAT_UP: {
            // Hex payload between the classifier and the first ';', e.g.
            // -00a66ef1...4800;rs=1;ss=A2;  A downlink is exactly a short
            // (18 B) or long (34 B) frame; anything else is line corruption,
            // dropped here so the decoder only sees plausible frames. Uplink
            // length is unchecked until the M5 decoder defines what it takes.
            const char *body = line + 1;
            const char *semi = strchr(body, ';');
            size_t hlen = semi ? (size_t)(semi - body) : strlen(body);
            if (hlen >= PONG_HEX_MAX) hlen = PONG_HEX_MAX - 1;
            if (f.kind == PONG_LINE_UAT_DOWN && hlen != 36 && hlen != 68) {
                g_status.pong_errors++;
                return;
            }
            memcpy(f.hex, body, hlen);
            f.hex[hlen] = '\0';
            f.hex_len = (uint16_t)hlen;

            // Scan ';'-separated suffix fields — captures carry any subset
            // of rs=/ss=, in any order. "rs=" is decimal (Reed-Solomon
            // symbols corrected, diagnostics only). "ss=" is a HEX int8
            // log-detector reading where 0x80 (-128) flags an errored
            // measurement — leave ss_valid clear for it. Never parse ss as
            // decimal (Stratux's Atoi path is a known bug, Appendix A).
            for (const char *p = semi; p; p = strchr(p + 1, ';')) {
                if (strncmp(p + 1, "rs=", 3) == 0) {
                    f.rs = (uint16_t)strtoul(p + 4, NULL, 10);
                } else if (strncmp(p + 1, "ss=", 3) == 0) {
                    unsigned long v = strtoul(p + 4, NULL, 16);
                    if ((v & 0xFF) != 0x80) {
                        f.ss = (uint16_t)v;
                        f.ss_valid = 1;
                    }
                }
            }

            if (g_settings.uat_en) uat_decode_frame(&f);
            return;
        }

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
    static uint8_t chunk[256];
    size_t len = 0;

    for (;;) {
        // Chunked reads for 3 Mbaud throughput: drain whatever the driver
        // ring has buffered (up to a chunk), waiting at most 20 ms for the
        // first byte so partial lines are delivered promptly. 256 bytes is
        // ~0.85 ms of line time at 3 Mbaud against the 4 KB driver ring.
        int n = uart_read_bytes(PONG_ACTIVE_PORT, chunk, sizeof(chunk),
                                pdMS_TO_TICKS(20));
        if (n <= 0) continue;

        for (int i = 0; i < n; i++) {
            uint8_t byte = chunk[i];
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
}
