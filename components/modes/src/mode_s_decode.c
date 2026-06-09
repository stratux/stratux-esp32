// Slim 1090ES Mode-S message decoder for stratux-esp32.
//
// Ported from dump1090 (FlightAware fork) mode_s.c — see components/modes/NOTICE
// for provenance and licensing (GPLv2-or-later). The getbit/getbits helpers are
// copied verbatim from dump1090 mode_s.h. The field-extraction math in the
// Extended-Squitter decoders is preserved; the modesMessage struct is slimmed to
// ms_msg_t and the DSP/scoring/error-correction/comm-b layers are dropped.

#include "mode_s_decode.h"
#include "crc.h"
#include "ais_charset.h"

#include <string.h>
#include <math.h>
#include <assert.h>

#define MODES_LONG_MSG_BYTES   14
#define MODES_SHORT_MSG_BYTES   7
#define MODES_LONG_MSG_BITS   112
#define MODES_SHORT_MSG_BITS   56
#define INVALID_ALTITUDE   (-9999)

// ---- bit extraction (verbatim from dump1090 mode_s.h) ----------------------

// The first bit (MSB of the first byte) is numbered 1, for consistency with how
// the specs number them.
static inline unsigned getbit(const unsigned char *data, unsigned bitnum)
{
    unsigned bi = bitnum - 1;
    unsigned by = bi >> 3;
    unsigned mask = 1 << (7 - (bi & 7));

    return (data[by] & mask) != 0;
}

static inline unsigned getbits(const unsigned char *data, unsigned firstbit, unsigned lastbit)
{
    unsigned fbi = firstbit - 1;
    unsigned lbi = lastbit - 1;
    unsigned nbi = (lastbit - firstbit + 1);

    unsigned fby = fbi >> 3;
    unsigned lby = lbi >> 3;
    unsigned nby = (lby - fby) + 1;

    unsigned shift = 7 - (lbi & 7);
    unsigned topmask = 0xFF >> (fbi & 7);

    assert (fbi <= lbi);
    assert (nbi <= 32);
    assert (nby <= 5);

    if (nby == 5) {
        return
            ((data[fby] & topmask) << (32 - shift)) |
            (data[fby + 1] << (24 - shift)) |
            (data[fby + 2] << (16 - shift)) |
            (data[fby + 3] << (8 - shift)) |
            (data[fby + 4] >> shift);
    } else if (nby == 4) {
        return
            ((data[fby] & topmask) << (24 - shift)) |
            (data[fby + 1] << (16 - shift)) |
            (data[fby + 2] << (8 - shift)) |
            (data[fby + 3] >> shift);
    } else if (nby == 3) {
        return
            ((data[fby] & topmask) << (16 - shift)) |
            (data[fby + 1] << (8 - shift)) |
            (data[fby + 2] >> shift);
    } else if (nby == 2) {
        return
            ((data[fby] & topmask) << (8 - shift)) |
            (data[fby + 1] >> shift);
    } else if (nby == 1) {
        return
            (data[fby] & topmask) >> shift;
    } else {
        return 0;
    }
}

// ---- field decode helpers (ported from dump1090 mode_s.c) ------------------

static int modesMessageLenByType(int type) {
    return (type & 0x10) ? MODES_LONG_MSG_BITS : MODES_SHORT_MSG_BITS;
}

// Gillham (Mode A -> Mode C) altitude conversion. dump1090 uses a 4096-entry
// lookup table built from mode_ac.c; modern ADS-B/Mode-S almost always uses the
// 25 ft Q-bit encoding, so for M1 we skip the rare Gillham 100 ft path rather
// than vendor the whole table. TODO(M1+): port mode_ac.c if Gillham-coded
// altitudes are seen in the wild.
static int modeAToModeC(unsigned modeA) {
    (void)modeA;
    return INVALID_ALTITUDE;
}

// Squawk (identity) Gillham de-interleave -> hex-coded octal.
static int decodeID13Field(int ID13Field) {
    int hexGillham = 0;
    if (ID13Field & 0x1000) {hexGillham |= 0x0010;} // Bit 12 = C1
    if (ID13Field & 0x0800) {hexGillham |= 0x1000;} // Bit 11 = A1
    if (ID13Field & 0x0400) {hexGillham |= 0x0020;} // Bit 10 = C2
    if (ID13Field & 0x0200) {hexGillham |= 0x2000;} // Bit  9 = A2
    if (ID13Field & 0x0100) {hexGillham |= 0x0040;} // Bit  8 = C4
    if (ID13Field & 0x0080) {hexGillham |= 0x4000;} // Bit  7 = A4
    if (ID13Field & 0x0020) {hexGillham |= 0x0100;} // Bit  5 = B1
    if (ID13Field & 0x0010) {hexGillham |= 0x0001;} // Bit  4 = D1 or Q
    if (ID13Field & 0x0008) {hexGillham |= 0x0200;} // Bit  3 = B2
    if (ID13Field & 0x0004) {hexGillham |= 0x0002;} // Bit  2 = D2
    if (ID13Field & 0x0002) {hexGillham |= 0x0400;} // Bit  1 = B4
    if (ID13Field & 0x0001) {hexGillham |= 0x0004;} // Bit  0 = D4
    return hexGillham;
}

// 13-bit AC altitude field (DF0/4/16/20).
static int decodeAC13Field(int AC13Field) {
    int m_bit  = AC13Field & 0x0040; // set = meters, clear = feet
    int q_bit  = AC13Field & 0x0010; // set = 25 ft encoding, clear = Gillham

    if (!m_bit) {
        if (q_bit) {
            int n = ((AC13Field & 0x1F80) >> 2) |
                    ((AC13Field & 0x0020) >> 1) |
                     (AC13Field & 0x000F);
            return ((n * 25) - 1000);
        } else {
            int n = modeAToModeC(decodeID13Field(AC13Field));
            if (n < -12)
                return INVALID_ALTITUDE;
            return (100 * n);
        }
    } else {
        // meters not supported
        return INVALID_ALTITUDE;
    }
}

// 12-bit AC altitude field (DF17/18 airborne position).
static int decodeAC12Field(int AC12Field) {
    int q_bit  = AC12Field & 0x10; // Bit 48 = Q

    if (q_bit) {
        int n = ((AC12Field & 0x0FE0) >> 1) |
                 (AC12Field & 0x000F);
        return ((n * 25) - 1000);
    } else {
        int n = ((AC12Field & 0x0FC0) << 1) |
                 (AC12Field & 0x003F);
        n = modeAToModeC(decodeID13Field(n));
        if (n < -12)
            return INVALID_ALTITUDE;
        return (100 * n);
    }
}

// 7-bit surface ground-movement field (ADS-B v0 PWL scale), midpoint kt.
static float decodeMovementFieldV0(unsigned movement) {
    if      (movement  >= 125) return 0;
    else if (movement  == 124) return 180;
    else if (movement  >= 109) return 100 + (movement - 109 + 0.5) * 5;
    else if (movement  >=  94) return 70 + (movement - 94 + 0.5) * 2;
    else if (movement  >=  39) return 15 + (movement - 39 + 0.5) * 1;
    else if (movement  >=  13) return 2 + (movement - 13 + 0.5) * 0.50;
    else if (movement  >=   9) return 1 + (movement - 9 + 0.5) * 0.25;
    else if (movement  >=   2) return 0.125 + (movement - 2 + 0.5) * 0.125;
    else                       return 0;
}

// NIC (navigation integrity category) from ME type code + NIC supplement-B.
// Table mirrors Stratux traffic.go parseDump1090Message().
static uint8_t nicFromType(unsigned metype, unsigned suppl_b) {
    switch (metype) {
        case 0: case 8: case 18: case 22: return 0;
        case 17: return 1;
        case 16: return suppl_b ? 3 : 2;
        case 15: return 4;
        case 14: return 5;
        case 13: return 6;
        case 12: return 7;
        case 11: return suppl_b ? 9 : 8;
        case 10: case 21: return 10;
        case 9:  case 20: return 11;
        default: return 0;
    }
}

// ---- Extended Squitter sub-decoders ----------------------------------------

static void decodeESIdentAndCategory(ms_msg_t *mm, const unsigned char *me)
{
    mm->mesub = getbits(me, 6, 8);

    mm->callsign[0] = ais_charset[getbits(me, 9, 14)];
    mm->callsign[1] = ais_charset[getbits(me, 15, 20)];
    mm->callsign[2] = ais_charset[getbits(me, 21, 26)];
    mm->callsign[3] = ais_charset[getbits(me, 27, 32)];
    mm->callsign[4] = ais_charset[getbits(me, 33, 38)];
    mm->callsign[5] = ais_charset[getbits(me, 39, 44)];
    mm->callsign[6] = ais_charset[getbits(me, 45, 50)];
    mm->callsign[7] = ais_charset[getbits(me, 51, 56)];
    mm->callsign[8] = 0;
    mm->callsign_valid = 1;

    for (unsigned i = 0; i < 8; ++i) {
        char c = mm->callsign[i];
        if (!(c >= 'A' && c <= 'Z') && !(c >= '0' && c <= '9') && c != ' ') {
            mm->callsign_valid = 0;
            break;
        }
    }

    mm->category = ((0x0E - mm->metype) << 4) | mm->mesub;
    mm->category_valid = 1;
}

static void decodeESAirborneVelocity(ms_msg_t *mm, const unsigned char *me, int check_imf)
{
    mm->mesub = getbits(me, 6, 8);

    if (mm->mesub < 1 || mm->mesub > 4)
        return;

    if (check_imf && getbit(me, 9))
        mm->non_icao = 1;

    switch (mm->mesub) {
    case 1: case 2:
        {
            unsigned ew_raw = getbits(me, 15, 24);
            unsigned ns_raw = getbits(me, 26, 35);

            if (ew_raw && ns_raw) {
                int ew_vel = (ew_raw - 1) * (getbit(me, 14) ? -1 : 1) * ((mm->mesub == 2) ? 4 : 1);
                int ns_vel = (ns_raw - 1) * (getbit(me, 25) ? -1 : 1) * ((mm->mesub == 2) ? 4 : 1);

                mm->gs = sqrtf((ns_vel * ns_vel) + (ew_vel * ew_vel) + 0.5f);
                mm->gs_valid = 1;

                if (mm->gs > 0) {
                    float ground_track = atan2f(ew_vel, ns_vel) * 180.0f / (float)M_PI;
                    if (ground_track < 0)
                        ground_track += 360;
                    mm->heading = ground_track;
                    mm->heading_valid = 1;
                }
            }
            break;
        }

    case 3: case 4:
        // Airspeed/heading subtypes: heading is magnetic-or-true and airspeed is
        // not ground speed, so we don't populate gs/track from these. (dump1090
        // keeps ias/tas; the GDL90 traffic report has no airspeed field.)
        break;
    }

    unsigned vert_rate = getbits(me, 38, 46);
    unsigned vert_rate_is_baro = getbit(me, 36);
    if (vert_rate) {
        int rate = (vert_rate - 1) * (getbit(me, 37) ? -64 : 64);
        if (vert_rate_is_baro) {
            mm->baro_rate = rate;
            mm->baro_rate_valid = 1;
        } else {
            mm->geom_rate = rate;
            mm->geom_rate_valid = 1;
        }
    }
}

static void decodeESSurfacePosition(ms_msg_t *mm, const unsigned char *me, int check_imf)
{
    mm->airground = MS_AG_GROUND;
    mm->cpr_valid = 1;
    mm->cpr_type = MS_CPR_SURFACE;

    unsigned movement = getbits(me, 6, 12);
    if (movement > 0 && movement < 125) {
        mm->gs_valid = 1;
        mm->gs = decodeMovementFieldV0(movement);
    }

    if (getbit(me, 13)) {
        mm->heading_valid = 1;
        mm->heading = getbits(me, 14, 20) * 360.0f / 128.0f;
    }

    if (check_imf && getbit(me, 21))
        mm->non_icao = 1;

    mm->cpr_odd = getbit(me, 22);
    mm->cpr_lat = getbits(me, 23, 39);
    mm->cpr_lon = getbits(me, 40, 56);

    mm->nic = nicFromType(mm->metype, 0);
    mm->nic_valid = 1;
}

static void decodeESAirbornePosition(ms_msg_t *mm, const unsigned char *me, int check_imf)
{
    // 6-7: surveillance status (alert/SPI) — not carried into the traffic table.

    // 8: IMF (DF18 TIS-B/ADS-R) or NIC supplement-B (DF17/DF18-ADSB).
    if (check_imf) {
        if (getbit(me, 8))
            mm->non_icao = 1;
    } else {
        mm->nic_suppl_b = getbit(me, 8);
    }

    unsigned AC12Field = getbits(me, 9, 20);

    if (mm->metype != 0) {
        mm->cpr_lat = getbits(me, 23, 39);
        mm->cpr_lon = getbits(me, 40, 56);

        // Catch a known all-zero failure mode (type 15, alt 0, lon 0, lat LSBs 0).
        if (AC12Field == 0 && mm->cpr_lon == 0 && (mm->cpr_lat & 0x0fff) == 0 && mm->metype == 15) {
            // drop: don't mark cpr_valid
        } else {
            mm->cpr_valid = 1;
            mm->cpr_type = MS_CPR_AIRBORNE;
            mm->cpr_odd = getbit(me, 22);
        }

        mm->nic = nicFromType(mm->metype, mm->nic_suppl_b);
        mm->nic_valid = 1;
    }

    if (AC12Field && mm->airground != MS_AG_GROUND) {
        int alt = decodeAC12Field(AC12Field);
        if (alt != INVALID_ALTITUDE) {
            if (mm->airground == MS_AG_INVALID)
                mm->airground = MS_AG_UNCERTAIN;

            if (mm->metype == 20 || mm->metype == 21 || mm->metype == 22) {
                mm->altitude_geom = alt;
                mm->altitude_geom_valid = 1;
            } else {
                mm->altitude_baro = alt;
                mm->altitude_baro_valid = 1;
            }
        }
    }
}

static void decodeESTestMessage(ms_msg_t *mm, const unsigned char *me)
{
    mm->mesub = getbits(me, 6, 8);
    if (mm->mesub == 7) { // (see 1090-WP-15-20)
        int ID13Field = getbits(me, 9, 21);
        if (ID13Field) {
            mm->squawk_valid = 1;
            mm->squawk = decodeID13Field(ID13Field);
        }
    }
}

static void decodeESOperationalStatus(ms_msg_t *mm, const unsigned char *me, int check_imf)
{
    mm->mesub = getbits(me, 6, 8);

    if (check_imf && getbit(me, 56))
        mm->non_icao = 1;

    if (mm->mesub == 0 || mm->mesub == 1) {
        unsigned version = getbits(me, 41, 43);
        if (version == 1 || version == 2) {
            mm->nac_p = getbits(me, 45, 48);
            mm->nac_p_valid = 1;
        }
    }
}

static void decodeExtendedSquitter(ms_msg_t *mm, const unsigned char *frame)
{
    const unsigned char *me = &frame[4];
    unsigned metype = mm->metype = getbits(me, 1, 5);
    unsigned check_imf = 0;

    // DF18: the Control Field selects the ES format and whether to look for IMF.
    if (mm->df == 18) {
        switch (mm->cf) {
        case 0: // ADS-B from non-transponder device, AA = 24-bit ICAO
            break;
        case 1: // anonymous / ground vehicle / fixed obstruction address
            mm->non_icao = 1;
            break;
        case 2: // Fine TIS-B
            check_imf = 1;
            break;
        case 3: // Coarse TIS-B airborne position+velocity (only IMF examined)
            if (getbit(me, 1))
                mm->non_icao = 1;
            return;
        case 5: // Fine TIS-B, non-ICAO 24-bit address
            mm->non_icao = 1;
            break;
        case 6: // Rebroadcast of ADS-B (ADS-R)
            check_imf = 1;
            break;
        default:
            mm->non_icao = 1;
            return;
        }
    }

    switch (metype) {
    case 1: case 2: case 3: case 4:
        decodeESIdentAndCategory(mm, me);
        break;
    case 19:
        decodeESAirborneVelocity(mm, me, check_imf);
        break;
    case 5: case 6: case 7: case 8:
        decodeESSurfacePosition(mm, me, check_imf);
        break;
    case 0:
    case 9: case 10: case 11: case 12: case 13: case 14: case 15: case 16: case 17: case 18:
    case 20: case 21: case 22:
        decodeESAirbornePosition(mm, me, check_imf);
        break;
    case 23:
        decodeESTestMessage(mm, me);
        break;
    case 31:
        decodeESOperationalStatus(mm, me, check_imf);
        break;
    default:
        break;
    }
}

// ---- public entry points ---------------------------------------------------

int ms_hex_to_bytes(const char *hex, size_t hexlen, uint8_t *out, size_t cap)
{
    if (hexlen == 0 || (hexlen & 1))
        return -1;
    size_t n = hexlen / 2;
    if (n > cap)
        n = cap;
    for (size_t i = 0; i < n; i++) {
        int hi, lo;
        char c = hex[2 * i];
        hi = (c >= '0' && c <= '9') ? c - '0'
           : (c >= 'A' && c <= 'F') ? c - 'A' + 10
           : (c >= 'a' && c <= 'f') ? c - 'a' + 10 : -1;
        c = hex[2 * i + 1];
        lo = (c >= '0' && c <= '9') ? c - '0'
           : (c >= 'A' && c <= 'F') ? c - 'A' + 10
           : (c >= 'a' && c <= 'f') ? c - 'a' + 10 : -1;
        if (hi < 0 || lo < 0)
            return -1;
        out[i] = (uint8_t)((hi << 4) | lo);
    }
    return (int)n;
}

void ms_decode_init(void)
{
    // fixBits = 0: build only the CRC lookup table; no error-correction tables
    // (we reject rather than repair — the Pong has already FEC-corrected).
    modesChecksumInit(0);
}

int ms_decode(ms_msg_t *mm, const uint8_t *frame, int len)
{
    if (!mm || !frame)
        return -1;

    memset(mm, 0, sizeof(*mm));

    if (len < MODES_SHORT_MSG_BYTES)
        return -1;

    unsigned df = getbits(frame, 1, 5);
    int msgbits = modesMessageLenByType(df);
    if (len * 8 < msgbits)
        return -1; // long-form DF needs all 14 bytes

    mm->df = df;
    uint32_t residual = modesChecksum(frame, msgbits);

    switch (df) {
    case 17:   // Extended squitter (ADS-B)
    case 18: { // Extended squitter / non-transponder (ADS-R / TIS-B)
        if (residual != 0)
            return -2; // failed CRC — reject (no bit-flip correction)
        mm->crc_ok = 1;
        mm->address_reliable = 1;
        mm->addr = getbits(frame, 9, 32) & 0xFFFFFF;
        if (df == 17) {
            mm->ca = getbits(frame, 6, 8);
            mm->airground = (mm->ca == 4) ? MS_AG_GROUND
                          : (mm->ca == 5) ? MS_AG_AIRBORNE
                          : MS_AG_UNCERTAIN;
        } else {
            mm->cf = getbits(frame, 6, 8);
            // CF4 (TIS-B/ADS-R management) and CF7 (reserved) carry no traffic
            // and their AA field is not an aircraft address — reject so the
            // caller never seeds the ICAO filter or the table from them.
            if (mm->cf == 4 || mm->cf == 7)
                return -1;
        }
        decodeExtendedSquitter(mm, frame);
        return 0;
    }

    case 11: // All-call reply: address in clear, CRC = parity/interrogator.
        // Low 7 bits may hold the interrogator ID; treat as reliable only when
        // the rest of the syndrome is zero.
        if ((residual & 0xFFFF80) != 0)
            return -2;
        mm->address_reliable = 1;
        mm->addr = getbits(frame, 9, 32) & 0xFFFFFF;
        mm->ca = getbits(frame, 6, 8);
        mm->airground = (mm->ca == 4) ? MS_AG_GROUND
                      : (mm->ca == 5) ? MS_AG_AIRBORNE
                      : MS_AG_UNCERTAIN;
        return 0; // no traffic payload, but confirms the address (ICAO filter)

    case 0:  // short air-air surveillance
    case 4:  // surveillance, altitude reply
    case 16: // long air-air surveillance
        // Address/Parity: address is inferred from the CRC syndrome and cannot
        // be verified — caller must confirm against the ICAO filter.
        mm->addr = residual & 0xFFFFFF;
        mm->address_reliable = 0;
        {
            unsigned AC = getbits(frame, 20, 32);
            if (AC) {
                int alt = decodeAC13Field(AC);
                if (alt != INVALID_ALTITUDE) {
                    mm->altitude_baro = alt;
                    mm->altitude_baro_valid = 1;
                }
            }
        }
        if (df == 0 || df == 16) {
            // VS (vertical status): bit 6 set => on ground
            mm->airground = getbit(frame, 6) ? MS_AG_GROUND : MS_AG_UNCERTAIN;
        }
        return 0;

    case 20: // Comm-B, altitude reply
        mm->addr = residual & 0xFFFFFF;
        mm->address_reliable = 0;
        {
            unsigned AC = getbits(frame, 20, 32);
            if (AC) {
                int alt = decodeAC13Field(AC);
                if (alt != INVALID_ALTITUDE) {
                    mm->altitude_baro = alt;
                    mm->altitude_baro_valid = 1;
                }
            }
        }
        // TODO(M1+): Comm-B BDS2,0 callsign would need decodeCommB (not ported).
        return 0;

    case 5:  // surveillance, identity reply
    case 21: // Comm-B, identity reply
        mm->addr = residual & 0xFFFFFF;
        mm->address_reliable = 0;
        {
            unsigned ID = getbits(frame, 20, 32);
            if (ID) {
                mm->squawk = decodeID13Field(ID);
                mm->squawk_valid = 1;
            }
        }
        return 0;

    default:
        return -1; // DF we don't handle
    }
}
