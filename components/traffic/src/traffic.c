#include "traffic.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "traffic";

// Comfortably fits internal RAM: ~300 * sizeof(traffic_info_t) (~30 KB).
#define TRAFFIC_MAX        300
#define TRAFFIC_AGE_OUT_MS 60000   // drop entries not heard for 60 s (tune at M1)

static traffic_info_t   s_table[TRAFFIC_MAX];
static size_t           s_count;
static SemaphoreHandle_t s_mutex;

void traffic_init(void)
{
    s_mutex = xSemaphoreCreateMutex();
    s_count = 0;
    memset(s_table, 0, sizeof(s_table));
    ESP_LOGI(TAG, "traffic table ready (cap=%d)", TRAFFIC_MAX);
}

void traffic_upsert(const traffic_info_t *t)
{
    if (!s_mutex || !t) return;
    xSemaphoreTake(s_mutex, portMAX_DELAY);

    // TODO(M1): find existing entry by (icao_addr, addr_type) and merge fields
    // (a position update should not clobber a known callsign, etc.). For now,
    // a linear scan + replace/append placeholder.
    traffic_info_t *slot = NULL;
    for (size_t i = 0; i < s_count; i++) {
        if (s_table[i].icao_addr == t->icao_addr &&
            s_table[i].addr_type == t->addr_type) {
            slot = &s_table[i];
            break;
        }
    }
    if (!slot && s_count < TRAFFIC_MAX) slot = &s_table[s_count++];
    if (slot) {
        *slot = *t;
        slot->last_seen_ms = esp_timer_get_time() / 1000;
    }

    xSemaphoreGive(s_mutex);
}

void traffic_mgr_task(void *arg)
{
    (void)arg;
    const TickType_t period = pdMS_TO_TICKS(100);   // ~10 Hz
    for (;;) {
        int64_t now_ms = esp_timer_get_time() / 1000;

        xSemaphoreTake(s_mutex, portMAX_DELAY);
        // TODO(M1): extrapolate position from track/speed since last_seen;
        //   compute bearing/distance from ownship (M3); cross-band dedup (M2).
        // Age-out: compact the array, dropping stale entries.
        size_t w = 0;
        for (size_t i = 0; i < s_count; i++) {
            if (now_ms - s_table[i].last_seen_ms <= TRAFFIC_AGE_OUT_MS) {
                if (w != i) s_table[w] = s_table[i];
                w++;
            }
        }
        s_count = w;
        xSemaphoreGive(s_mutex);

        vTaskDelay(period);
    }
}

size_t traffic_snapshot(traffic_info_t *out, size_t max)
{
    if (!s_mutex || !out) return 0;
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    size_t n = s_count < max ? s_count : max;
    memcpy(out, s_table, n * sizeof(traffic_info_t));
    xSemaphoreGive(s_mutex);
    return n;
}
