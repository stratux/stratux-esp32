#pragma once
#include <stddef.h>
#include <stdint.h>

// GDL90 message IDs we emit (verified in Stratux gen_gdl90.go). The encoder
// builds these; the framer below wraps them.
#define GDL90_MSG_HEARTBEAT        0x00   // M0
#define GDL90_MSG_UPLINK           0x07   // M5 (FIS-B uplink relay)
#define GDL90_MSG_OWNSHIP          0x0A   // M3
#define GDL90_MSG_OWNSHIP_GEO_ALT  0x0B   // M3
#define GDL90_MSG_TRAFFIC          0x14   // M1
#define GDL90_MSG_FF_ID_AHRS       0x65   // M4 (ForeFlight ID 0x00 / AHRS 0x01)
#define GDL90_MSG_AHRS_LEVIL       0x4C   // M4
#define GDL90_MSG_STRATUX_HB       0xCC   // M1 (Stratux custom heartbeat/status)

// CRC-16 per GDL90 ICD §2.2.4 — NOT standard CRC-XMODEM (poly 0x1021, init 0,
// data XORed into the LOW byte AFTER 8 shifts). Self-test vector:
// "123456789" -> 0xBEEF (XMODEM would give 0x31C3). Senders MUST match this
// byte-for-byte or every frame CRC-fails. Copied from connext
// aera660_gdl90_uart.c::gdl90_crc16 — framing/CRC are direction-agnostic.
uint16_t gdl90_crc16(const uint8_t *data, size_t len);

// Frame a raw payload into a complete GDL90 message:
//   0x7E | payload | CRC16-low | CRC16-high | 0x7E
// with control-byte escaping applied AFTER the CRC (0x7E->0x7D 0x5E,
// 0x7D->0x7D 0x5D, i.e. byte ^ 0x20). Returns framed length, or -1 if `out`
// (capacity `out_cap`) is too small.
int gdl90_frame(const uint8_t *payload, size_t len, uint8_t *out, size_t out_cap);

// Run the CRC self-test (logs + aborts the emitter path on failure) and prepare
// scratch buffers. Call once before gdl90_emit_task.
void gdl90_out_init(void);

// FreeRTOS task (Stratux sendGDL90): 1 Hz heartbeat (0x00) + Stratux status
// (0xCC); N Hz traffic (0x14) from traffic_snapshot(); frame each and hand to
// net_gdl90_send(). M0 emits only the heartbeat (with the "UTC OK" bit CLEAR
// until a real time source exists — see M0 / time source in AGENTS.md).
void gdl90_emit_task(void *arg);
