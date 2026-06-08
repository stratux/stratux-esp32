#pragma once
#include <stdint.h>

// Shared Pong frame type — lives in `common` (not the `pong` component) so the
// decoders (`modes`, `uat`) can consume it without depending on `pong`, which
// in turn calls them (that would be a circular component dependency).
// Protocol: Stratux main/pong.go (see AGENTS.md "Protocol gotchas").

// First-character classification of each newline-delimited Pong line.
typedef enum {
    PONG_LINE_UNKNOWN = 0,
    PONG_LINE_1090ES,    // '*'  Mode-S frame (AVR/hex) + ss=
    PONG_LINE_UAT_DOWN,  // '-'  978 UAT downlink + rs=/ss=
    PONG_LINE_UAT_UP,    // '+'  978 UAT uplink   + rs=/ss=
    PONG_LINE_HEARTBEAT, // '.'  heartbeat (Pong does NOT heartbeat-timeout)
    PONG_LINE_STATUS,    // '\'' quoted ASCII status text
    PONG_LINE_LOG,       // other ASCII; may contain "ERROR SPI" (Pong fault)
} pong_line_kind_t;

// One demodulated frame ready for a decoder. `hex` holds the raw payload with
// the leading classifier and trailing ss=/rs= fields stripped.
typedef struct {
    pong_line_kind_t kind;
    char     hex[64];   // AVR/hex payload, NUL-terminated
    uint16_t hex_len;   // bytes of `hex` used
    // ss is a HEX log-detector reading (non-linear) for Pong — parse as HEX,
    // NOT decimal. UAT ss is a hex int8 where 0x80 (-128) flags an errored
    // measurement. Do NOT copy Stratux's decimal Atoi path (Appendix A).
    uint16_t ss;        // signal strength (raw hex reading)
    uint16_t rs;        // Reed-Solomon errors corrected (UAT only); 0 otherwise
} pong_frame_t;
