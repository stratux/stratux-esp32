// Slim 1090ES Mode-S message decoder for stratux-esp32.
//
// Ported from dump1090 (FlightAware fork) mode_s.c / mode_s.h — see
// components/modes/NOTICE for provenance and licensing (GPLv2-or-later).
//
// This decodes the *message layer* only: the Pong radio delivers
// already-demodulated, FEC-corrected ASCII hex frames, so the DSP front-end,
// message scoring, brute-force error correction, comm-b/MRAR/nav/opstatus
// extras, and the SDR/threading layers from dump1090 are intentionally absent.
// We compute the 24-bit CRC and accept DF17/18 only when the residual is zero
// (no bit-flip correction — trust the Pong's FEC).

#pragma once
#include <stdint.h>
#include <stddef.h>

// CPR encoding type (subset of dump1090's cpr_type_t — no COARSE here).
typedef enum {
    MS_CPR_SURFACE,
    MS_CPR_AIRBORNE,
} ms_cpr_type_t;

// Air/ground state (mirrors dump1090 airground_t).
typedef enum {
    MS_AG_INVALID,
    MS_AG_GROUND,
    MS_AG_AIRBORNE,
    MS_AG_UNCERTAIN,
} ms_airground_t;

// Decoded Mode-S message. Only the fields the traffic layer consumes are kept.
// "*_valid" flags gate every optional field; a freshly memset-0 struct means
// "nothing decoded".
typedef struct {
    uint8_t  df;             // Downlink Format (bits 1-5)
    uint8_t  crc_ok;         // 1 if CRC residual was zero (DF17/18 only)

    uint32_t addr;           // 24-bit announced address (AA), masked to 24 bits
    uint8_t  non_icao;       // address is non-ICAO (DF18 anonymous / IMF=1)
    uint8_t  address_reliable; // 1 if addr is CRC-verified (DF11/17/18); 0 if
                               // inferred from Address/Parity (DF0/4/5/16/20/21)
                               // and must be confirmed against an ICAO filter

    uint8_t  ca;             // Capability (DF17) — air/ground hint
    uint8_t  cf;             // Control Field (DF18) — ADS-B/ADS-R/TIS-B selector

    uint8_t  metype;         // DF17/18 ME type code (TC)
    uint8_t  mesub;          // DF17/18 ME subtype

    ms_airground_t airground;

    // Identity / category (TC 1-4)
    char     callsign[9];    // NUL-terminated; valid iff callsign_valid
    uint8_t  callsign_valid;
    uint8_t  category;       // emitter category byte
    uint8_t  category_valid;

    // Squawk (DF5/21 identity, or TC23 sub7) — hex-coded octal
    uint16_t squawk;
    uint8_t  squawk_valid;

    // Altitude (feet)
    int32_t  altitude_baro;  int32_t altitude_baro_valid;
    int32_t  altitude_geom;  int32_t altitude_geom_valid;

    // Velocity (TC19) / surface movement (TC5-8)
    float    gs;             uint8_t gs_valid;       // ground speed, kt
    float    heading;        uint8_t heading_valid;  // ground track, deg true
    int32_t  baro_rate;      uint8_t baro_rate_valid; // ft/min
    int32_t  geom_rate;      uint8_t geom_rate_valid; // ft/min

    // Raw CPR (TC 5-8 surface, TC 9-22 airborne)
    uint8_t  cpr_valid;
    uint8_t  cpr_odd;
    ms_cpr_type_t cpr_type;
    uint32_t cpr_lat;        // 17-bit CPR latitude
    uint32_t cpr_lon;        // 17-bit CPR longitude
    uint8_t  nic_suppl_b;    // NIC supplement-B bit (airborne pos bit 8)

    // Integrity / accuracy
    uint8_t  nic;            // navigation integrity category (computed from TC)
    uint8_t  nic_valid;
    uint8_t  nac_p;          // navigation accuracy (TC31 operational status)
    uint8_t  nac_p_valid;
} ms_msg_t;

// Build the Mode-S CRC lookup table. Call once before ms_decode().
void ms_decode_init(void);

// Decode a raw frame (already hex-decoded to bytes). len is the byte count
// (7 for short / 14 for long; intermediate lengths are zero-padded by the
// caller). Returns 0 on a usable decode, <0 on reject (bad length / unknown DF
// / failed CRC for DF17/18 / DF18 management-CF). On success *mm is fully
// populated.
int ms_decode(ms_msg_t *mm, const uint8_t *frame, int len);

// Convert an ASCII hex string to bytes (at most `cap`). Returns the byte
// count, or -1 on an odd/zero length or a non-hex character. Shared by the
// firmware line parser and the host replay/test tools so they accept exactly
// the same input.
int ms_hex_to_bytes(const char *hex, size_t hexlen, uint8_t *out, size_t cap);
