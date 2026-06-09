// Per-aircraft CPR even/odd staging — see cpr_store.h. stratux-esp32 original.

#include "cpr_store.h"
#include "cpr.h"
#include <string.h>

#define CPR_STORE_MAX        200
#define CPR_PAIR_WINDOW_MS 10000   // even+odd must be within 10 s for a global fix
#define CPR_REL_TTL_MS     60000   // reference position usable for relative decode

typedef struct {
    uint8_t  used;
    uint32_t addr;

    uint8_t  even_valid;
    uint8_t  even_surface;         // CPR type of the staged even frame
    uint32_t even_lat, even_lon;
    int64_t  even_ms;

    uint8_t  odd_valid;
    uint8_t  odd_surface;          // CPR type of the staged odd frame
    uint32_t odd_lat, odd_lon;
    int64_t  odd_ms;

    uint8_t  pos_valid;            // a position has been resolved
    double   lat, lon;
    int64_t  pos_ms;

    int64_t  last_ms;              // last time this slot was touched
} cpr_entry_t;

static cpr_entry_t s_tab[CPR_STORE_MAX];

// Find the slot for addr, or claim one (evicting the least-recently-touched
// slot if the table is full).
static cpr_entry_t *slot_for(uint32_t addr)
{
    cpr_entry_t *free_slot = NULL;
    cpr_entry_t *oldest = &s_tab[0];

    for (int i = 0; i < CPR_STORE_MAX; i++) {
        cpr_entry_t *e = &s_tab[i];
        if (e->used && e->addr == addr)
            return e;
        if (!e->used && !free_slot)
            free_slot = e;
        if (e->last_ms < oldest->last_ms)
            oldest = e;
    }

    cpr_entry_t *e = free_slot ? free_slot : oldest;
    memset(e, 0, sizeof(*e));
    e->used = 1;
    e->addr = addr;
    return e;
}

int cpr_store_update(uint32_t addr, const ms_msg_t *mm, int64_t now_ms,
                     double *out_lat, double *out_lon)
{
    if (!mm || !mm->cpr_valid)
        return 0;

    cpr_entry_t *e = slot_for(addr);
    e->last_ms = now_ms;

    int surface = (mm->cpr_type == MS_CPR_SURFACE);
    int fflag = mm->cpr_odd ? 1 : 0;

    // Stash this frame as the latest even/odd sample.
    if (mm->cpr_odd) {
        e->odd_valid = 1;
        e->odd_surface = (uint8_t)surface;
        e->odd_lat = mm->cpr_lat;
        e->odd_lon = mm->cpr_lon;
        e->odd_ms = now_ms;
    } else {
        e->even_valid = 1;
        e->even_surface = (uint8_t)surface;
        e->even_lat = mm->cpr_lat;
        e->even_lon = mm->cpr_lon;
        e->even_ms = now_ms;
    }

    double rlat = 0, rlon = 0;

    // 1) Global decode from a fresh even/odd pair. The pair must be the same
    // CPR type: surface frames use a 90° encoding, airborne 360°, so a mixed
    // pair (routine during takeoff/landing) decodes to garbage that can still
    // pass the decoder's internal zone checks (dump1090 enforces the same).
    if (e->even_valid && e->odd_valid && e->even_surface == e->odd_surface) {
        int64_t dt = e->even_ms > e->odd_ms ? (e->even_ms - e->odd_ms)
                                            : (e->odd_ms - e->even_ms);
        if (dt <= CPR_PAIR_WINDOW_MS) {
            int rc;
            if (surface) {
                if (e->pos_valid && (now_ms - e->pos_ms) <= CPR_REL_TTL_MS)
                    rc = decodeCPRsurface(e->lat, e->lon,
                                          e->even_lat, e->even_lon,
                                          e->odd_lat, e->odd_lon,
                                          fflag, &rlat, &rlon);
                else
                    rc = -1; // no reference position for surface global decode
            } else {
                rc = decodeCPRairborne(e->even_lat, e->even_lon,
                                       e->odd_lat, e->odd_lon,
                                       fflag, &rlat, &rlon);
            }
            if (rc == 0) {
                e->pos_valid = 1;
                e->lat = rlat;
                e->lon = rlon;
                e->pos_ms = now_ms;
                *out_lat = rlat;
                *out_lon = rlon;
                return 1;
            }
        }
    }

    // 2) Relative decode against a recent resolved position (single frame).
    if (e->pos_valid && (now_ms - e->pos_ms) <= CPR_REL_TTL_MS) {
        if (decodeCPRrelative(e->lat, e->lon, mm->cpr_lat, mm->cpr_lon,
                              fflag, surface, &rlat, &rlon) == 0) {
            e->lat = rlat;
            e->lon = rlon;
            e->pos_ms = now_ms;
            *out_lat = rlat;
            *out_lon = rlon;
            return 1;
        }
    }

    return 0;
}
