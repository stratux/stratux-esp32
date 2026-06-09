#pragma once
#include <stdint.h>

// Shared Pong frame type — lives in `common` (not the `pong` component) so the
// decoders (`modes`, `uat`) can consume it without depending on `pong`, which
// in turn calls them (that would be a circular component dependency).
// Protocol: Stratux main/pong.go (see AGENTS.md "Protocol gotchas").

// Largest payload we must hold: a full UAT uplink (FIS-B) frame is 432 bytes =
// 864 hex chars; real captures top out at 877-char lines (incl. prefix +
// trailing ";rs=..;ss=..;"). Size both the line buffer and hex[] to clear that
// with headroom — the old 64-byte hex[] truncated ~2/3 of uplink frames.
#define PONG_HEX_MAX  896

// First-character classification of each newline-delimited Pong line.
typedef enum {
    PONG_LINE_UNKNOWN = 0,
    PONG_LINE_1090ES,    // '*'  Mode-S frame (AVR/hex); optional trailing ss=
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
    char     hex[PONG_HEX_MAX];   // AVR/hex payload, NUL-terminated
    uint16_t hex_len;             // bytes of `hex` used
    // ss is a HEX log-detector reading (non-linear) for Pong — parse as HEX,
    // NOT decimal. UAT ss is a hex int8 where 0x80 (-128) flags an errored
    // measurement. Do NOT copy Stratux's decimal Atoi path (Appendix A).
    // OPTIONAL: real 1090ES (*) lines may carry NO ss= at all (the trailing
    // ";<n>" seen in some firmware is a message counter, not ss). Treat ss as
    // present only when a literal "ss=" field is found; otherwise ss_valid=0.
    uint16_t ss;        // signal strength (raw hex reading); valid iff ss_valid
    uint16_t rs;        // Reed-Solomon errors corrected (UAT only); 0 otherwise
    uint8_t  ss_valid;  // 1 if an "ss=" field was present and parsed, else 0
} pong_frame_t;
