#include "traffic.h"
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "traffic";

#define TRAFFIC_AGE_OUT_MS 60000   // drop entries not heard for 60 s (tune at M1)
#define TRAFFIC_EXTRAP_MIN_MS 1000 // dead-reckon at ~1 Hz granularity
#define TRAFFIC_EXTRAP_MAX_MS 30000 // stop dead-reckoning a fix older than this
#define TRAFFIC_POS_STALE_MS  15000 // demote position_valid once the (possibly
                                    // extrapolated) position is older than this

static traffic_info_t   s_table[TRAFFIC_TABLE_MAX];
static size_t           s_count;
static SemaphoreHandle_t s_mutex;

void traffic_init(void)
{
    s_mutex = xSemaphoreCreateMutex();
    s_count = 0;
    memset(s_table, 0, sizeof(s_table));
    ESP_LOGI(TAG, "traffic table ready (cap=%d)", TRAFFIC_TABLE_MAX);
}

// ICAO-addressed reports (direct ADS-B or a TIS-B/ADS-R rebroadcast) describe
// the same physical aircraft and must share one entry; non-ICAO/self-assigned
// addresses are a separate namespace that may numerically collide with ICAO.
static inline bool addr_is_icao(traffic_addr_type_t at)
{
    return at == ADDR_TYPE_ADSB_ICAO || at == ADDR_TYPE_TISB_ICAO;
}

void traffic_upsert(const traffic_info_t *t)
{
    if (!s_mutex || !t) return;
    xSemaphoreTake(s_mutex, portMAX_DELAY);

    const int64_t now_ms = esp_timer_get_time() / 1000;

    // Find an existing entry (by ICAO within the ICAO class, else by exact
    // (addr, addr_type)), else claim a new slot.
    traffic_info_t *slot = NULL;
    for (size_t i = 0; i < s_count; i++) {
        if (s_table[i].icao_addr != t->icao_addr)
            continue;
        if (s_table[i].addr_type == t->addr_type ||
            (addr_is_icao(s_table[i].addr_type) && addr_is_icao(t->addr_type))) {
            slot = &s_table[i];
            break;
        }
    }
    if (!slot && s_count < TRAFFIC_TABLE_MAX) {
        slot = &s_table[s_count++];
        memset(slot, 0, sizeof(*slot));
        slot->icao_addr = t->icao_addr;
        slot->addr_type = t->addr_type;
    }

    if (slot) {
        // Same ICAO heard both directly and via rebroadcast: keep the direct
        // ADS-B labeling (a TIS-B report must not downgrade it).
        if (t->addr_type == ADDR_TYPE_ADSB_ICAO)
            slot->addr_type = ADDR_TYPE_ADSB_ICAO;

        // Accumulate which bands have contributed (ES / UAT / both).
        slot->src = t->src;
        slot->src_bands |= t->src;

        // Field-by-field merge: a single 1090ES frame carries only part of the
        // state, so only overwrite the fields this report actually populated.
        // A position update should not erase a known callsign, and vice-versa.
        if (t->position_valid) {
            slot->lat = t->lat;
            slot->lng = t->lng;
            slot->position_valid = true;
            slot->position_ms = now_ms;
            slot->extrapolated = false;
            slot->extrap_ms = 0;   // re-anchor extrapolation on the fresh fix
        }
        if (t->tail_valid) {
            memcpy(slot->tail, t->tail, sizeof(slot->tail));
            slot->tail_valid = true;
        }
        if (t->cat_valid) {
            slot->emitter_cat = t->emitter_cat;
            slot->cat_valid = true;
        }
        if (t->alt_valid) {
            slot->alt_ft = t->alt_ft;
            slot->alt_valid = true;
        }
        if (t->gnss_alt_valid) {
            slot->gnss_alt_ft = t->gnss_alt_ft;
            slot->gnss_alt_valid = true;
        }
        if (t->airground_valid) {
            slot->on_ground = t->on_ground;
            slot->airground_valid = true;
        }
        if (t->speed_valid) {
            slot->speed_kt = t->speed_kt;
            slot->track_deg = t->track_deg;
            slot->speed_valid = true;
        }
        if (t->vvel_valid) {
            slot->vvel_fpm = t->vvel_fpm;
            slot->vvel_valid = true;
        }
        // NIC and NACp arrive in different messages (NIC with every position,
        // NACp only in TC31 operational status) — merge them independently so
        // a position frame can't clobber a real NACp and vice-versa.
        if (t->nic_valid) {
            slot->nic = t->nic;
            slot->nic_valid = true;
        }
        if (t->nacp_valid) {
            slot->nacp = t->nacp;
            slot->nacp_valid = true;
        }
        if (t->ss_valid) {
            slot->ss = t->ss;
            slot->ss_valid = true;
        }
        slot->last_seen_ms = now_ms;
    }

    xSemaphoreGive(s_mutex);
}

// Dead-reckon an airborne entry forward along its track. Advances incrementally
// from the last step (so error doesn't compound from the original fix faster
// than real updates correct it) and flags the position as extrapolated. Anchors
// on the time of the last real fix (position_ms), never on last_seen_ms — any
// frame type refreshes last_seen, which is liveness, not position freshness.
static void extrapolate(traffic_info_t *e, int64_t now_ms)
{
    if (!e->position_valid || !e->speed_valid || e->on_ground || e->speed_kt == 0)
        return;

    // Integer-ms gates before any floating point: this runs under the table
    // mutex shared with the decode path, and ESP32 doubles are soft-float.
    int64_t anchor = e->extrap_ms ? e->extrap_ms : e->position_ms;
    if (now_ms - anchor < TRAFFIC_EXTRAP_MIN_MS)
        return;
    if (now_ms - e->position_ms > TRAFFIC_EXTRAP_MAX_MS)
        return; // fix too old to keep dead-reckoning; mgr will demote it

    // Single-precision math: the ESP32 FPU is float-only, and dead-reckoning
    // over seconds needs nowhere near double accuracy.
    float dt_s = (float)(now_ms - anchor) / 1000.0f;
    float dist_nm = e->speed_kt * (dt_s / 3600.0f);
    float dist_deg = dist_nm / 60.0f;                 // 1 nm = 1/60 deg latitude
    float trk = e->track_deg * ((float)M_PI / 180.0f);
    float coslat = cosf((float)e->lat * ((float)M_PI / 180.0f));
    if (coslat < 1e-6f) coslat = 1e-6f;

    e->lat += dist_deg * cosf(trk);
    e->lng += dist_deg * sinf(trk) / coslat;
    if (e->lng > 180.0)  e->lng -= 360.0;
    if (e->lng < -180.0) e->lng += 360.0;

    e->extrap_ms = now_ms;
    e->extrapolated = true;
}

void traffic_mgr_task(void *arg)
{
    (void)arg;
    // 1 Hz: matches the extrapolation granularity and the GDL90 emit rate; the
    // 60 s age-out needs nothing finer, and each tick contends for the table
    // mutex with the decode hot path.
    const TickType_t period = pdMS_TO_TICKS(1000);
    for (;;) {
        int64_t now_ms = esp_timer_get_time() / 1000;

        xSemaphoreTake(s_mutex, portMAX_DELAY);
        // Extrapolate live positions, demote stale ones, then age-out by
        // compacting the array. Bearing/distance from ownship is TODO (M3).
        size_t w = 0;
        for (size_t i = 0; i < s_count; i++) {
            if (now_ms - s_table[i].last_seen_ms <= TRAFFIC_AGE_OUT_MS) {
                traffic_info_t *e = &s_table[i];
                extrapolate(e, now_ms);
                // A position that is neither fresh nor being extrapolated must
                // not be presented as a live target (the entry itself stays —
                // a new CPR fix revalidates it).
                if (e->position_valid) {
                    int64_t pos_ms = e->extrap_ms > e->position_ms
                                       ? e->extrap_ms : e->position_ms;
                    if (now_ms - pos_ms > TRAFFIC_POS_STALE_MS) {
                        e->position_valid = false;
                        e->extrapolated = false;
                    }
                }
                if (w != i) s_table[w] = *e;
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

size_t traffic_count(void)
{
    if (!s_mutex) return 0;
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    size_t n = s_count;
    xSemaphoreGive(s_mutex);
    return n;
}
