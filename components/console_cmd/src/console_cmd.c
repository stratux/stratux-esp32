#include "console_cmd.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "settings.h"
#include "net.h"

static const char *TAG = "console_cmd";

#define CMD_LINE_MAX 256
#define CMD_MAX_TOKS 8

// Replies share UART0 TX with the log stream. One printf() call per reply
// keeps the line atomic: stdout's lock serializes against ESP_LOGx at the
// write-call level, and every log line ends in '\n', so the '$' always lands
// at line start. (Never uart_write_bytes() here — raw driver writes interleave
// with the VFS console mid-line.)
#define REPLY(...) printf(__VA_ARGS__)

// Split a command line into tokens in place. Spaces separate tokens; within a
// token a double-quoted span keeps its spaces, with \" and \\ unescaped (so
// SSIDs/passwords may contain spaces and quotes). Quotes are stripped:
// ssid="a b" yields the token ssid=a b. Returns the count, or -1 on an
// unterminated quote.
static int tokenize(char *s, char *tok[], int max)
{
    int n = 0;
    while (*s) {
        while (*s == ' ') s++;
        if (!*s) break;
        if (n == max) return n;
        tok[n++] = s;
        char *w = s;                       // compact escapes/quotes in place
        bool quoted = false;
        while (*s && (quoted || *s != ' ')) {
            if (*s == '"') { quoted = !quoted; s++; continue; }
            if (quoted && *s == '\\' && (s[1] == '"' || s[1] == '\\')) s++;
            *w++ = *s++;
        }
        if (quoted) return -1;
        if (*s) s++;                       // step past the separating space
        *w = '\0';
    }
    return n;
}

// Find "key=" among tokens [from..n); returns the value part or NULL.
static const char *kv_find(char *tok[], int from, int n, const char *key)
{
    size_t klen = strlen(key);
    for (int i = from; i < n; i++) {
        if (strncmp(tok[i], key, klen) == 0 && tok[i][klen] == '=')
            return tok[i] + klen + 1;
    }
    return NULL;
}

static void cmd_wifi_get(void)
{
    net_sta_status_t st;
    net_sta_status(&st);
    REPLY("$OK sta_en=%d ssid=\"%s\" pass=%s ip=%s gw=%s dns=%s state=%s\n",
          g_settings.sta_en, g_settings.sta_ssid,
          g_settings.sta_pass[0] ? "***" : "\"\"",
          st.ip, st.gw, st.dns,
          !st.enabled ? "disabled" : (st.connected ? "connected" : "connecting"));
}

static void cmd_wifi_set(char *tok[], int n)
{
    const char *ssid   = kv_find(tok, 2, n, "ssid");
    const char *pass   = kv_find(tok, 2, n, "pass");
    const char *sta_en = kv_find(tok, 2, n, "sta_en");
    if (!ssid && !pass && !sta_en) {
        REPLY("$ERR no keys (want sta_en=/ssid=/pass=)\n");
        return;
    }

    // Stage into a copy, validate everything, then commit + save — the same
    // PATCH pattern as the web /setSettings handler.
    settings_t next = g_settings;
    if (sta_en) {
        if (strcmp(sta_en, "0") && strcmp(sta_en, "1")) {
            REPLY("$ERR sta_en must be 0 or 1\n");
            return;
        }
        next.sta_en = sta_en[0] == '1';
    }
    if (ssid) {
        size_t l = strlen(ssid);
        if (l > 32 || (l == 0 && next.sta_en)) {
            REPLY("$ERR ssid must be 1-32 chars (empty only when disabled)\n");
            return;
        }
        strlcpy(next.sta_ssid, ssid, sizeof(next.sta_ssid));
    }
    if (pass) {
        size_t l = strlen(pass);
        if (l != 0 && (l < 8 || l > 64)) {
            REPLY("$ERR pass must be empty (open) or 8-64 chars\n");
            return;
        }
        strlcpy(next.sta_pass, pass, sizeof(next.sta_pass));
    }
    if (next.sta_en && !next.sta_ssid[0]) {
        REPLY("$ERR sta_en=1 needs a ssid\n");
        return;
    }

    settings_t prev = g_settings;
    g_settings = next;
    if (settings_save() != ESP_OK) {
        g_settings = prev;
        REPLY("$ERR NVS save failed\n");
        return;
    }
    REPLY("$OK saved (reboot to apply)\n");
}

static void cmd_dest_get(void)
{
    REPLY("$OK dest=%s\n", g_settings.gdl90_dest);
}

static void cmd_dest_set(char *tok[], int n)
{
    const char *dest = kv_find(tok, 2, n, "dest");
    if (!dest) {
        REPLY("$ERR want dest=<ip[,ip...]> or dest=\"\"\n");
        return;
    }
    if (strlen(dest) >= sizeof(g_settings.gdl90_dest) ||
        net_set_static_dest(dest) < 0) {            // validates, applies live
        REPLY("$ERR bad ip list (max 4 dotted quads)\n");
        return;
    }
    settings_t prev = g_settings;
    strlcpy(g_settings.gdl90_dest, dest, sizeof(g_settings.gdl90_dest));
    if (settings_save() != ESP_OK) {
        g_settings = prev;
        net_set_static_dest(prev.gdl90_dest);
        REPLY("$ERR NVS save failed\n");
        return;
    }
    REPLY("$OK saved\n");
}

bool console_cmd_handle_line(const char *line)
{
    if (!line || line[0] != '$') return false;

    char buf[CMD_LINE_MAX];
    strlcpy(buf, line, sizeof(buf));
    size_t l = strlen(buf);                  // strip a trailing CR (host CRLF)
    if (l && buf[l - 1] == '\r') buf[l - 1] = '\0';

    char *tok[CMD_MAX_TOKS];
    int n = tokenize(buf, tok, CMD_MAX_TOKS);
    if (n < 1) {
        REPLY("$ERR bad line\n");
        return true;
    }

    if (strcmp(tok[0], "$WIFI") == 0 && n >= 2) {
        if (strcmp(tok[1], "GET") == 0)      { cmd_wifi_get(); return true; }
        if (strcmp(tok[1], "SET") == 0)      { cmd_wifi_set(tok, n); return true; }
    } else if (strcmp(tok[0], "$DEST") == 0 && n >= 2) {
        if (strcmp(tok[1], "GET") == 0)      { cmd_dest_get(); return true; }
        if (strcmp(tok[1], "SET") == 0)      { cmd_dest_set(tok, n); return true; }
    } else if (strcmp(tok[0], "$REBOOT") == 0) {
        REPLY("$OK rebooting\n");
        vTaskDelay(pdMS_TO_TICKS(250));      // let the reply drain the TX FIFO
        esp_restart();
    }

    REPLY("$ERR unknown command\n");
    return true;
}

#if CONFIG_PONG_SOURCE_CONSOLE

// Replay builds: pong_rx_task owns UART0 RX and routes '$' lines to
// console_cmd_handle_line() from its classifier — no reader of our own.
void console_cmd_start(void)
{
    ESP_LOGI(TAG, "command channel routed via pong console reader");
}

#else

#include "driver/uart.h"

#define CMD_UART      UART_NUM_0
#define CMD_RX_BUF    1024
#define CMD_CHUNK     64

// Own UART0 RX: read chunks, assemble lines, act on '$' commands, ignore the
// rest (same loop shape as pong_rx_task). Console pins and baud (115200) are
// already set up by the ROM/bootloader — installing the driver only adds RX
// buffering; do NOT uart_param_config/uart_set_pin here, that glitches the
// log stream. stdout keeps using the non-driver VFS path, so printf replies
// are unaffected.
static void console_cmd_task(void *arg)
{
    (void)arg;
    if (!uart_is_driver_installed(CMD_UART)) {
        ESP_ERROR_CHECK(uart_driver_install(CMD_UART, CMD_RX_BUF, 0, 0, NULL, 0));
    }

    static char line[CMD_LINE_MAX];
    size_t fill = 0;
    uint8_t chunk[CMD_CHUNK];

    ESP_LOGI(TAG, "command channel listening on UART0 ($WIFI/$DEST/$REBOOT)");

    for (;;) {
        int rd = uart_read_bytes(CMD_UART, chunk, sizeof(chunk),
                                 pdMS_TO_TICKS(100));
        for (int i = 0; i < rd; i++) {
            char c = (char)chunk[i];
            if (c == '\n') {
                line[fill] = '\0';
                if (fill) console_cmd_handle_line(line);
                fill = 0;
            } else if (fill < sizeof(line) - 1) {
                line[fill++] = c;
            } else {
                fill = 0;                    // overlong line: drop and resync
            }
        }
    }
}

void console_cmd_start(void)
{
    xTaskCreate(console_cmd_task, "console_cmd", 4096, NULL, 5, NULL);
}

#endif // CONFIG_PONG_SOURCE_CONSOLE
