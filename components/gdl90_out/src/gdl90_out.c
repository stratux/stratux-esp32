#include "gdl90_out.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "net.h"
#include "traffic.h"
#include "stratux_status.h"

static const char *TAG = "gdl90";

// Latched by gdl90_out_init(): true only if the CRC self-test produced 0xBEEF.
// gdl90_emit_task() refuses to run if this is false — a mis-built CRC would make
// every frame CRC-fail at the EFB, so silence is correct.
static bool s_crc_ok = false;

#define GDL90_FLAG  0x7E
#define GDL90_ESC   0x7D

// CRC-16 per GDL90 ICD §2.2.4 (poly 0x1021, init 0x0000) — see header banner.
// Tableless per-bit form (equivalent to the ICD table form); plenty fast at
// GDL90 rates on a 240 MHz core. Verbatim from connext aera660_gdl90_uart.c.
uint16_t gdl90_crc16(const uint8_t *data, size_t len)
{
    uint16_t crc = 0;
    for (size_t i = 0; i < len; i++) {
        for (int b = 0; b < 8; b++) {
            crc = (crc & 0x8000)
                  ? (uint16_t)((crc << 1) ^ 0x1021)
                  : (uint16_t)(crc << 1);
        }
        crc ^= (uint16_t)data[i];
    }
    return crc;
}

int gdl90_frame(const uint8_t *payload, size_t len, uint8_t *out, size_t out_cap)
{
    if (!payload || !out) return -1;
    uint16_t crc = gdl90_crc16(payload, len);
    const uint8_t crc_lo = (uint8_t)(crc & 0xFF);
    const uint8_t crc_hi = (uint8_t)(crc >> 8);

    size_t n = 0;
    #define PUT(b)  do { if (n >= out_cap) return -1; out[n++] = (uint8_t)(b); } while (0)
    PUT(GDL90_FLAG);
    // Escape every payload + CRC byte (CRC appended little-endian, low first).
    for (size_t i = 0; i < len + 2; i++) {
        uint8_t b = (i < len) ? payload[i] : (i == len ? crc_lo : crc_hi);
        if (b == GDL90_FLAG || b == GDL90_ESC) {
            PUT(GDL90_ESC);
            PUT(b ^ 0x20);
        } else {
            PUT(b);
        }
    }
    PUT(GDL90_FLAG);
    #undef PUT
    return (int)n;
}

void gdl90_out_init(void)
{
    // GDL90-variant self-test: "123456789" -> 0xBEEF (XMODEM would be 0x31C3).
    uint16_t v = gdl90_crc16((const uint8_t *)"123456789", 9);
    if (v != 0xBEEF) {
        s_crc_ok = false;
        ESP_LOGE(TAG, "CRC self-test FAILED: got 0x%04x, expected 0xBEEF — "
                      "GDL90 emit DISABLED (every frame would CRC-fail)", v);
        return;
    }
    s_crc_ok = true;
    ESP_LOGI(TAG, "CRC self-test ok: 0xBEEF (GDL90 ICD §2.2.4 variant)");
}

// Build the 7-byte GDL90 heartbeat (0x00). M0: status bits reflect g_status —
// crucially leave the "UTC OK" bit CLEAR and the timestamp zero until a real
// clock exists (M0). Full bit layout: Stratux gen_gdl90.go makeHeartbeat.
static size_t build_heartbeat(uint8_t *p)
{
    memset(p, 0, 7);
    p[0] = GDL90_MSG_HEARTBEAT;
    p[1] = 0x01;                       // Status byte 1: GPS pos valid? no -> bit set only when ready
    p[2] = g_status.utc_ok ? 0x01 : 0x00;  // Status byte 2 bit0 = UTC OK (CLEAR at M0)
    // p[3..4] timestamp (LE, seconds since 0000Z) — zero until UTC OK.
    // p[5..6] message counts.
    return 7;
}

void gdl90_emit_task(void *arg)
{
    (void)arg;
    if (!s_crc_ok) {
        ESP_LOGE(TAG, "CRC self-test failed at init; emitter idle (no frames sent)");
        vTaskDelete(NULL);
    }
    uint8_t payload[512];
    uint8_t frame[1024];

    TickType_t last_hb = xTaskGetTickCount();

    for (;;) {
        TickType_t now = xTaskGetTickCount();

        // 1 Hz heartbeat (0x00). M1 adds the Stratux 0xCC status here too.
        if (now - last_hb >= pdMS_TO_TICKS(1000)) {
            size_t plen = build_heartbeat(payload);
            int flen = gdl90_frame(payload, plen, frame, sizeof(frame));
            if (flen > 0) net_gdl90_send(frame, (size_t)flen);
            last_hb = now;
        }

        // TODO(M1): traffic_snapshot() -> build a 0x14 Traffic Report per entry
        // (gen_gdl90.go makeTrafficReport field layout) -> frame -> send.

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
