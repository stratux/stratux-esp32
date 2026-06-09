// Part of dump978, a UAT decoder.
//
// Copyright 2015, Oliver Jowett <oliver@mutability.co.uk>
//
// This file is free software: you may copy, redistribute and/or modify it
// under the terms of the GNU General Public License as published by the
// Free Software Foundation, either version 2 of the License, or (at your
// option) any later version.
//
// This file is distributed in the hope that it will be useful, but
// WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//
// stratux-esp32 adaptation: decode functions vendored from dump978
// uat_decode.c with const frame parameters; display/uplink paths removed
// (see components/uat/NOTICE).

#include <math.h>
#include <string.h>

#include "uat_decode.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static void uat_decode_hdr(const uint8_t *frame, struct uat_adsb_mdb *mdb)
{
    mdb->mdb_type = (frame[0] >> 3) & 0x1f;
    mdb->address_qualifier = (address_qualifier_t) (frame[0] & 0x07);
    mdb->address = (frame[1] << 16) | (frame[2] << 8) | frame[3];
}

static double dimensions_widths[16] = {
    11.5, 23, 28.5, 34, 33, 38, 39.5, 45, 45, 52, 59.5, 67, 72.5, 80, 80, 90
};

static void uat_decode_sv(const uint8_t *frame, struct uat_adsb_mdb *mdb)
{
    uint32_t raw_lat, raw_lon, raw_alt;

    mdb->has_sv = 1;

    mdb->nic = (frame[11] & 15);

    raw_lat = (frame[4] << 15) | (frame[5] << 7) | (frame[6] >> 1);
    raw_lon = ((frame[6] & 0x01) << 23) | (frame[7] << 15) | (frame[8] << 7) | (frame[9] >> 1);

    if (mdb->nic != 0 || raw_lat != 0 || raw_lon != 0) {
        mdb->position_valid = 1;
        mdb->lat = raw_lat * 360.0 / 16777216.0;
        if (mdb->lat > 90)
            mdb->lat -= 180;
        mdb->lon = raw_lon * 360.0 / 16777216.0;
        if (mdb->lon > 180)
            mdb->lon -= 360;
    }

    raw_alt = (frame[10] << 4) | ((frame[11] & 0xf0) >> 4);
    if (raw_alt != 0) {
        mdb->altitude_type = (frame[9] & 1) ? ALT_GEO : ALT_BARO;
        mdb->altitude = (raw_alt - 1) * 25 - 1000;
    }

    mdb->airground_state = (frame[12] >> 6) & 0x03;

    switch (mdb->airground_state) {
    case AG_SUBSONIC:
    case AG_SUPERSONIC:
        {
            int raw_ns, raw_ew, raw_vvel;

            raw_ns = ((frame[12] & 0x1f) << 6) | ((frame[13] & 0xfc) >> 2);
            if ((raw_ns & 0x3ff) != 0) {
                mdb->ns_vel_valid = 1;
                mdb->ns_vel = ((raw_ns & 0x3ff) - 1);
                if (raw_ns & 0x400)
                    mdb->ns_vel = 0 - mdb->ns_vel;
                if (mdb->airground_state == AG_SUPERSONIC)
                    mdb->ns_vel *= 4;
            }

            raw_ew = ((frame[13] & 0x03) << 9) | (frame[14] << 1) | ((frame[15] & 0x80) >> 7);
            if ((raw_ew & 0x3ff) != 0) {
                mdb->ew_vel_valid = 1;
                mdb->ew_vel = ((raw_ew & 0x3ff) - 1);
                if (raw_ew & 0x400)
                    mdb->ew_vel = 0 - mdb->ew_vel;
                if (mdb->airground_state == AG_SUPERSONIC)
                    mdb->ew_vel *= 4;
            }

            if (mdb->ns_vel_valid && mdb->ew_vel_valid) {
                if (mdb->ns_vel != 0 || mdb->ew_vel != 0) {
                    mdb->track_type = TT_TRACK;
                    mdb->track = (uint16_t)(360 + 90 - atan2(mdb->ns_vel, mdb->ew_vel) * 180 / M_PI) % 360;
                }

                mdb->speed_valid = 1;
                mdb->speed = (int) sqrt(mdb->ns_vel * mdb->ns_vel + mdb->ew_vel * mdb->ew_vel);
            }

            raw_vvel = ((frame[15] & 0x7f) << 4) | ((frame[16] & 0xf0) >> 4);
            if ((raw_vvel & 0x1ff) != 0) {
                mdb->vert_rate_source = (raw_vvel & 0x400) ? ALT_BARO : ALT_GEO;
                mdb->vert_rate = ((raw_vvel & 0x1ff) - 1) * 64;
                if (raw_vvel & 0x200)
                    mdb->vert_rate = 0 - mdb->vert_rate;
            }
        }
        break;

    case AG_GROUND:
        {
            int raw_gs, raw_track;

            raw_gs = ((frame[12] & 0x1f) << 6) | ((frame[13] & 0xfc) >> 2);
            if (raw_gs != 0) {
                mdb->speed_valid = 1;
                mdb->speed = ((raw_gs & 0x3ff) - 1);
            }

            raw_track = ((frame[13] & 0x03) << 9) | (frame[14] << 1) | ((frame[15] & 0x80) >> 7);
            switch ((raw_track & 0x0600)>>9) {
            case 1: mdb->track_type = TT_TRACK; break;
            case 2: mdb->track_type = TT_MAG_HEADING; break;
            case 3: mdb->track_type = TT_TRUE_HEADING; break;
            }

            if (mdb->track_type != TT_INVALID)
                mdb->track = (raw_track & 0x1ff) * 360 / 512;

            mdb->dimensions_valid = 1;
            mdb->length = 15 + 10 * ((frame[15] & 0x38) >> 3);
            mdb->width = dimensions_widths[(frame[15] & 0x78) >> 3];
            mdb->position_offset = (frame[15] & 0x04) ? 1 : 0;
        }
        break;

    case AG_RESERVED:
        // nothing
        break;
    }

    if ((frame[0] & 7) == 2 || (frame[0] & 7) == 3) {
        mdb->utc_coupled = 0;
        mdb->tisb_site_id = (frame[16] & 0x0f);
    } else {
        mdb->utc_coupled = (frame[16] & 0x08) ? 1 : 0;
        mdb->tisb_site_id = 0;
    }
}

static char base40_alphabet[40] = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ  ..";

static void uat_decode_ms(const uint8_t *frame, struct uat_adsb_mdb *mdb)
{
    uint16_t v;
    int i;

    mdb->has_ms = 1;

    v = (frame[17]<<8) | (frame[18]);
    mdb->emitter_category = (v/1600) % 40;
    mdb->callsign[0] = base40_alphabet[(v/40) % 40];
    mdb->callsign[1] = base40_alphabet[v % 40];
    v = (frame[19]<<8) | (frame[20]);
    mdb->callsign[2] = base40_alphabet[(v/1600) % 40];
    mdb->callsign[3] = base40_alphabet[(v/40) % 40];
    mdb->callsign[4] = base40_alphabet[v % 40];
    v = (frame[21]<<8) | (frame[22]);
    mdb->callsign[5] = base40_alphabet[(v/1600) % 40];
    mdb->callsign[6] = base40_alphabet[(v/40) % 40];
    mdb->callsign[7] = base40_alphabet[v % 40];
    mdb->callsign[8] = 0;

    // trim trailing spaces
    for (i = 7; i >= 0; --i) {
        if (mdb->callsign[i] == ' ')
            mdb->callsign[i] = 0;
        else
            break;
    }

    mdb->emergency_status = (frame[23] >> 5) & 7;
    mdb->uat_version = (frame[23] >> 2) & 7;
    mdb->sil = (frame[23] & 3);
    mdb->transmit_mso = (frame[24] >> 2) & 0x3f;
    mdb->nac_p = (frame[25] >> 4) & 15;
    mdb->nac_v = (frame[25] >> 1) & 7;
    mdb->nic_baro = (frame[25] & 1);
    mdb->has_cdti = (frame[26] & 0x80 ? 1 : 0);
    mdb->has_acas = (frame[26] & 0x40 ? 1 : 0);
    mdb->acas_ra_active = (frame[26] & 0x20 ? 1 : 0);
    mdb->ident_active = (frame[26] & 0x10 ? 1 : 0);
    mdb->atc_services = (frame[26] & 0x08 ? 1 : 0);
    mdb->heading_type = (frame[26] & 0x04 ? HT_MAGNETIC : HT_TRUE);
    if (mdb->callsign[0])
        mdb->callsign_type = (frame[26] & 0x02 ? CS_CALLSIGN : CS_SQUAWK);
}

static void uat_decode_auxsv(const uint8_t *frame, struct uat_adsb_mdb *mdb)
{
    int raw_alt = (frame[29] << 4) | ((frame[30] & 0xf0) >> 4);
    if (raw_alt != 0) {
        mdb->sec_altitude = (raw_alt - 1) * 25 - 1000;
        mdb->sec_altitude_type = (frame[9] & 1) ? ALT_BARO : ALT_GEO;
    } else {
        mdb->sec_altitude_type = ALT_INVALID;
    }

    mdb->has_auxsv = 1;
}

void uat_decode_adsb_mdb(const uint8_t *frame, struct uat_adsb_mdb *mdb)
{
    static struct uat_adsb_mdb mdb_zero;

    *mdb = mdb_zero;

    uat_decode_hdr(frame, mdb);

    switch (mdb->mdb_type) {
    case 0: // HDR SV
    case 4: // HDR SV (TC+0) (TS)
    case 7: // HDR SV reserved
    case 8: // HDR SV reserved
    case 9: // HDR SV reserved
    case 10: // HDR SV reserved
        uat_decode_sv(frame, mdb);
        break;

    case 1: // HDR SV MS AUXSV
        uat_decode_sv(frame, mdb);
        uat_decode_ms(frame, mdb);
        uat_decode_auxsv(frame, mdb);
        break;

    case 2: // HDR SV AUXSV
    case 5: // HDR SV (TC+1) AUXSV
    case 6: // HDR SV (TS) AUXSV
        uat_decode_sv(frame, mdb);
        uat_decode_auxsv(frame, mdb);
        break;

    case 3: // HDR SV MS (TS)
        uat_decode_sv(frame, mdb);
        uat_decode_ms(frame, mdb);
        break;

    default:
        break;
    }
}

// stratux-esp32 addition (not from dump978) — see uat_decode.h.
int uat_hex_to_bytes(const char *hex, size_t hexlen, uint8_t *out, size_t cap)
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
