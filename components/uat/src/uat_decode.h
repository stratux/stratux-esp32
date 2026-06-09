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
// stratux-esp32 adaptation (see components/uat/NOTICE): downlink (ADS-B MDB)
// decode only — the display functions, name tables, and the uplink/FIS-B path
// (M5) are not ported. Frame parameters are const. No stdio dependency, so
// the module compiles standalone for host tests (tools/test_uat.c).

#ifndef UAT_DECODE_H
#define UAT_DECODE_H

#include <stdint.h>
#include <stddef.h>

// DO-282 downlink frame sizes (payload after the Pong's RS/FEC pass).
#define UAT_SHORT_FRAME_BYTES 18   // HDR + SV               (36 hex chars)
#define UAT_LONG_FRAME_BYTES  34   // HDR + SV + MS/AUXSV    (68 hex chars)

//
// Datatypes
//

typedef enum { AQ_ADSB_ICAO=0, AQ_NATIONAL=1, AQ_TISB_ICAO=2, AQ_TISB_OTHER=3, AQ_VEHICLE=4,
               AQ_FIXED_BEACON=5, AQ_RESERVED_6=6, AQ_RESERVED_7=7 } address_qualifier_t;
typedef enum { ALT_INVALID=0, ALT_BARO, ALT_GEO } altitude_type_t;
typedef enum { AG_SUBSONIC=0, AG_SUPERSONIC=1, AG_GROUND=2, AG_RESERVED=3 } airground_state_t;
typedef enum { TT_INVALID=0, TT_TRACK, TT_MAG_HEADING, TT_TRUE_HEADING } track_type_t;
typedef enum { HT_INVALID=0, HT_MAGNETIC, HT_TRUE } heading_type_t;
typedef enum { CS_INVALID=0, CS_CALLSIGN, CS_SQUAWK } callsign_type_t;

struct uat_adsb_mdb {
    // presence bits
    int has_sv : 1;
    int has_ms : 1;
    int has_auxsv : 1;

    int position_valid : 1;
    int ns_vel_valid : 1;
    int ew_vel_valid : 1;
    int speed_valid : 1;
    int dimensions_valid : 1;

    //
    // HDR
    //
    uint8_t mdb_type;
    address_qualifier_t address_qualifier;
    uint32_t address;

    //
    // SV
    //

    // if position_valid:
    double lat;
    double lon;

    altitude_type_t altitude_type;
    int32_t altitude; // in feet

    uint8_t nic;

    airground_state_t airground_state;

    // if ns_vel_valid:
    int16_t ns_vel; // in kts
    // if ew_vel_valid:
    int16_t ew_vel; // in kts

    track_type_t track_type;
    uint16_t track;

    // if speed_valid:
    uint16_t speed; // in kts

    altitude_type_t vert_rate_source;
    int16_t vert_rate; // in ft/min

    // if lengthwidth_valid:
    double length; // in meters (just to be different)
    double width;  // in meters (just to be different)
    int position_offset : 1;  // true if Position Offset Applied

    int utc_coupled : 1;      // true if UTC Coupled flag is set (ADS-B)
    uint8_t tisb_site_id;     // TIS-B site ID, or zero in ADS-B messages

    //
    // MS
    //

    uint8_t emitter_category;
    callsign_type_t callsign_type;
    char callsign[9];
    uint8_t emergency_status;
    uint8_t uat_version;
    uint8_t sil;
    uint8_t transmit_mso;
    uint8_t nac_p;
    uint8_t nac_v;
    uint8_t nic_baro;

    // capabilities:
    int has_cdti : 1;
    int has_acas : 1;
    // operational modes:
    int acas_ra_active : 1;
    int ident_active : 1;
    int atc_services : 1;

    heading_type_t heading_type;

    //
    // AUXSV

    altitude_type_t sec_altitude_type;
    int32_t sec_altitude; // in feet
};

//
// Decode prototypes
//

// `frame` must hold UAT_LONG_FRAME_BYTES readable bytes (zero-pad a short
// frame) — the caller gates MS/AUXSV-bearing MDB types on the long length.
void uat_decode_adsb_mdb(const uint8_t *frame, struct uat_adsb_mdb *mdb);

// stratux-esp32 addition: hex string -> bytes (same semantics as the modes
// component's ms_hex_to_bytes, duplicated so this module stays standalone).
// Returns bytes written (capped at `cap`), or -1 on odd/zero length or a
// non-hex character.
int uat_hex_to_bytes(const char *hex, size_t hexlen, uint8_t *out, size_t cap);

#endif
