# AGENTS.md — working guide for stratux-esp32

Guidance for AI agents and humans working in this repo: build/flash, layout,
architecture, the hardware/protocol gotchas, conventions, and milestone status.
This is the authoritative day-to-day reference.

## What this is

A port of [Stratux](https://github.com/stratux/stratux) (a Go ADS-B receiver
daemon for the Raspberry Pi) to the **ESP32** (LilyGO **TTGO T8**), fed by an
integrated **uAvionix Pong** radio and emitting **GDL90 over WiFi** to EFB apps
(ForeFlight, Garmin Pilot, FltPlan Go). The Pong delivers *already-demodulated*
1090ES + 978 UAT frames as ASCII lines over serial, so there is **no SDR/DSP** on
the device — the pipeline is:

```
Pong UART → line parser → frame decoders → traffic table → GDL90 encoder → WiFi
```

**Status: M1 complete (verified 2026-06-11).** The traffic pipeline works
end-to-end: Pong line parsing, 1090ES Mode-S decode (dump1090 port: CRC, ICAO
filter, CPR, TC1–4/19/31, DF18 ADS-R/TIS-B), UAT downlink decode (dump978
port), traffic table with extrapolation/aging, and GDL90 0x00/0xCC/0x14 unicast
per DHCP lease on :4000 — confirmed against a real 22-min capture replayed over
console UART0 (`docs/TESTING.md` has the procedure and results). Caveat: the
UAT downlink path is validated only by synthetic unit tests — the capture had
no 978 traffic; real-capture validation is deferred (TESTING.md item 1).
**M2 is code-complete** (settings → NVS, web UI with live traffic WS + FIS-B
product breakdown + raw-Pong diag, cross-band source tracking) — on-device
browser verification pending (docs/TESTING.md M2 section). M3+ are stubs.

## Build / flash / monitor

ESP-IDF **v5.5.x** (pin to the version `connext-emulator` uses). The default and
primary build is the **WiFi** variant.

```bash
. $IDF_PATH/export.sh
idf.py -B build-wifi -D RADIO=wifi build
idf.py -B build-wifi -p <PORT> -b 115200 flash monitor    # NOTE: 115200, not the default
idf.py -B build-bt   -D RADIO=bt   build                  # M6 Connext/BT — placeholder only
```

- **Always flash at `-b 115200`.** The TTGO T8's USB-serial bridge is unreliable
  at the default 460800 and flashing fails intermittently above 115200.
- `RADIO` selects the sdkconfig overlay (`sdkconfig.defaults.<radio>`) via the
  top-level `CMakeLists.txt`. `wifi` is the default; `bt` is a reserved M6 stub.
- Editing `sdkconfig.defaults*` only takes effect for keys not already in the
  generated `sdkconfig`; delete `sdkconfig` to re-apply defaults.
- The web UI (`www/index.html`) is **embedded in the app image**
  (`EMBED_TXTFILES` in `components/web/CMakeLists.txt`) — rebuild + reflash the
  app to update it. The FAT `storage` partition stays reserved for when the UI
  outgrows a single file, but note the current sdkconfig sets
  `CONFIG_FATFS_LFN_NONE` (8.3 names — cannot even hold "index.html"); enable
  LFN before reviving the FAT route.

There is no compiler/linter configured outside an `idf.py` build, so editor
(clang) "file not found" / "undeclared identifier" diagnostics for `esp_*`,
`freertos/*`, `driver/*`, etc. are **expected noise** — those headers resolve
only inside the build. Don't chase them.

## Repository layout

```
stratux-esp32/
├── CMakeLists.txt              radio-selection wrapper (RADIO=wifi default)
├── partitions.csv              4 MB, single-app, no OTA; trailing FAT = web assets
├── sdkconfig.defaults[.wifi|.bt]   common + per-radio overlays
├── main/
│   ├── CMakeLists.txt
│   └── main.c                  app_main: NVS → WiFi → tasks
├── components/
│   ├── common/                 SHARED: pins.h, settings.{c,h}, status, pong_frame.h
│   ├── pong/                   UART2 reader + line classifier → decoders
│   ├── modes/                  1090ES DF17/18 decoder (NET-NEW, M1)
│   ├── uat/                    UAT decoder (downlink M1; FIS-B M5)
│   ├── traffic/                TrafficInfo table, extrapolation, aging
│   ├── gdl90_out/              GDL90 builders + HDLC/CRC framer (CRC is real)
│   ├── net/                    WiFi SoftAP + UDP :4000 delivery
│   ├── web/                    esp_http_server + WS (M2)
│   └── wxstore/                M5 weather store — empty placeholder
├── www/                        static web UI (→ FAT storage partition)
├── tools/                      flash/monitor/bridge helpers (placeholders)
└── docs/
```

**Note on structure:** `pins.h` and `settings.{c,h}` live in
**`components/common/`**, not `main/`. ESP-IDF components cannot depend on the
`main` component, but the worker components need the pin map and settings — so
shared headers live in `common`, which everything may `REQUIRES`. Likewise the
shared `pong_frame_t` type is in `common/pong_frame.h` so the decoders
(`modes`/`uat`) don't have to depend on `pong` (which calls them — that would be
a circular dependency).

## Architecture & task map

Go goroutines → FreeRTOS tasks; Go channels → FreeRTOS queues; Go mutexes →
FreeRTOS mutexes. Keep one mutex for the traffic table and one for the
situation/status snapshot. Tasks spawned in `app_main`:

| Task (`main.c`)   | Component   | Responsibility |
|---|---|---|
| `pong_rx_task`    | `pong`      | Read UART2, classify lines, dispatch to decoders |
| (called inline)   | `modes`     | Decode 1090ES DF17/18 → `traffic_upsert()` |
| (called inline)   | `uat`       | Decode UAT downlink → traffic; uplink → FIS-B (M5) |
| `traffic_mgr_task`| `traffic`   | Extrapolate, age-out, bearing/distance, dedup |
| `gdl90_emit_task` | `gdl90_out` | Build + frame GDL90, hand to net layer |
| (event loop)      | `net`       | WiFi SoftAP, UDP :4000 to clients |
| `web_start()`     | `web`       | esp_http_server + WS for the web UI |

Decode is **synchronous in `pong_rx_task`** in the skeleton. If 3 Mbaud bursts
cause UART backpressure, move decode to a queue-fed task.

## Hardware gotchas (do not relearn the hard way)

From the sibling `connext-emulator`'s hardware findings (real TTGO T8 silicon):

- **Flash at 115200** (see above).
- **PSRAM is split:** 8 MB physical but only **4 MB heap-mappable** on original
  ESP32 silicon; the upper 4 MB is reachable only via the bank-switched **himem**
  API (32 KB windows). Plan large buffers (weather, M5) around this.
- **Keep the Pong UART off GPIO16/17.** Those are PSRAM data pins; a UART pinned
  there is silently stranded. The pin plan already avoids them.
- **Log with `ESP_LOGx` directly.** A deep custom log-callback seam caused an
  early boot loop in connext (since resolved). Avoid that pattern.
- **Bad microSD detection:** counterfeit/bad cards return garbage CID/CSD (e.g. a
  phantom "121 MB" identity). Treat `sdmmc_card_init()`/SSR timeouts as a card
  failure, not a bus-speed tuning problem.

### Pin plan (TTGO T8 — `components/common/include/pins.h`)

| Function | GPIO | Notes |
|---|---|---|
| Pong UART2 RX (Pong→ESP32) | **35** | input-only; NO internal pull — Pong must drive idle-high |
| Pong UART2 TX (ESP32→Pong) | **33** | RTC GPIO |
| Pong RTS (static level)    | **32** | drive to `ClearRTS()` level — NOT HW flow control |
| Console UART0 TX/RX        | 1 / 3  | USB-serial — leave alone |
| microSD SDMMC 1-bit        | 14/15/2 | reserve if SD populated (M5 Tier-3) |
| PSRAM data                 | 16/17  | **reserved — do not use** |
| Flash                      | 6–11   | **reserved** |
| I²C (M3/M4)                | 21/22  | GPS aux / IMU / OLED |
| GPS UART (M3)              | 34/4   | 34 is input-only (RX) |

Avoid strapping pins (0, 2, 5, 12, 15) for new outputs; GPIO4 is free. Input-only
pins 34–39 have no internal pulls.

## Protocol gotchas

**Pong serial link** (Stratux `main/pong.go`): 3,000,000 baud, 8N1.
Newline-delimited ASCII; first char classifies: `*`=1090ES, `-`=UAT downlink,
`+`=UAT uplink, `.`=heartbeat, `'`=status, other=log (may contain `ERROR SPI`).
- **The `.` heartbeat is idle-only**: the Pong sends it only when no message
  went out in the 5 s heartbeat period, so it disappears while traffic flows.
  Link liveness must key on **any** line (frames included) with a silence
  timeout longer than the heartbeat period — an alive Pong says something at
  least every ~5 s. Never wait for `.` specifically.
- Host **clears RTS once** and never toggles it; there is **no CTS / flow
  control**. Drive GPIO32 as a static level — do not enable `UART_HW_FLOWCTRL_RTS`
  unless bench testing proves the Pong pauses/resumes on it.
- **Parse `ss=` (signal strength) as HEX**, not decimal — it is a non-linear log
  detector reading. UAT `ss` is a hex int8 where `0x80` (−128) flags an errored
  measurement. Do **not** copy Stratux's decimal `Atoi` path. `rs=` is
  Reed-Solomon errors corrected (diagnostics only).
- **`ERROR SPI`:** Stratux only *logs* "Restarting Pong" — it does nothing. This
  port must implement real recovery (reopen UART / toggle a reset GPIO / mark the
  device degraded).

**GDL90 CRC — NOT XMODEM** (`gdl90_out.c`): GDL90 ICD §2.2.4 variant — poly
`0x1021`, init `0x0000`, data XORed into the **low** byte *after* 8 shifts.
Self-test vector `"123456789" → 0xBEEF` (XMODEM would give `0x31C3`). Senders must
match byte-for-byte or every frame CRC-fails. Framing:
`0x7E | payload | CRC16(little-endian) | 0x7E`, with `0x7E→0x7D 0x5E` /
`0x7D→0x7D 0x5D` escaping applied **after** the CRC. `gdl90_crc16()` and
`gdl90_frame()` already implement this; keep the self-test in `gdl90_out_init()`.

**Time source — no RTC.** The ESP32 has no battery-backed clock. Until GPS (M3),
NTP, or a manual time push sets the clock, emit the GDL90 heartbeat with the
**"UTC OK" bit CLEAR** and a zero timestamp rather than lying about time.

**EFB UDP delivery — per-lease unicast (matches Stratux).** No dnsmasq on the
ESP32, but the SoftAP's own DHCP server is the lease source. `net.c` tracks
clients via `IP_EVENT_AP_STAIPASSIGNED` / `WIFI_EVENT_AP_STADISCONNECTED` and
unicasts each GDL90 datagram to every lease on `:4000` — the analog of Stratux
`network.go`'s `getDHCPLeases()` + per-client `DialUDP`. Do **not** broadcast:
802.11 sends broadcast/multicast at the lowest basic rate, buffered until DTIM,
so power-saving EFB tablets drop them. The AP IP is pinned to `192.168.10.1/24`
so leases land on the subnet EFBs expect. Sleep/throttle detection (Stratux
`isSleeping`/`isThrottled`) is a later refinement, not correctness.

## Configuration (NVS)

Replaces the Pi's `stratux.conf`. Namespace `"stratux"`, struct in
`components/common/include/settings.h` (`g_settings`). Defaults: SSID `stratux`,
open AP, channel 1, AP IP `192.168.10.1`, both bands enabled, region `US` — so
existing EFB setups "just work." Read/written via the web UI (`/getSettings`,
`/setSettings`) at M2.

## Conventions

- C, ESP-IDF component model. One component per concern; `REQUIRES` lists only
  what a component includes/links. Components never depend on `main`.
- Shared types/config go in `components/common` (pins, settings, status,
  `pong_frame_t`).
- Reuse from `connext-emulator/esp32/` where it fits — partitions, sdkconfig, the
  GDL90 framer/CRC, WiFi SoftAP + `esp_http_server`, and (M5) `wxstore`.
- Port from Stratux Go (`main/pong.go`, `gen_gdl90.go`, `traffic.go`,
  `network.go`, `uatparse/`) — but verify, don't trust, the non-authoritative Go
  paths (e.g. the decimal `ss` parse, which is wrong for hex Pong values).
- **Licensing:** Stratux is GPL; dump1090/dump978 and the Pong frame format have
  their own terms. Confirm before vendoring any decoder source.

## Milestones

- **M0** ✅ Bring-up: SoftAP up, GDL90 heartbeat (0x00) on :4000, EFB shows connected.
- **M1** ✅ *(UAT-downlink off-air validation deferred — docs/TESTING.md)*
  Traffic: Pong → 1090ES + UAT-downlink decode → traffic table → GDL90
  0x14 + Stratux 0xCC. *The Mode-S decoder is the main net-new work* — needs
  CRC/parity, ICAO + non-ICAO, even/odd CPR with expiry, airborne + surface
  position, velocity (TC 19), identity (TC 1–4), NIC/NACp, DF18 ADS-R/TIS-B.
  Prefer porting dump1090's `mode_s.c` message layer over hand-rolling.
- **M2** ✅ *(code-complete; browser verification pending — docs/TESTING.md)*
  Web UI + robustness: settings → NVS, embedded single-page UI with live
  traffic WS + FIS-B uplink product breakdown + raw-Pong diag (`/getPongLog`),
  cross-band source tracking (the ICAO-keyed table already dedups across
  bands; `src_bands` makes it visible). FAT static-asset serving deferred.
  Per-lease unicast delivery already landed in M0.
- **M3** GPS/ownship (0x0A/0x0B) — also supplies the time source ("UTC OK").
- **M4** AHRS (0x65 sub-id 0x01 / Levil 0x4C).
- **M5** UAT uplink: a) passthrough as GDL90 0x07; b) embedded FIS-B + `wxstore`
  (needs the WROVER overlay).
- **M6** Optional: Connext/BT variant, LoRa/OGN, OTA.

## Key references

- Sibling `connext-emulator/esp32/` — reusable ESP-IDF baseline: the real
  `partitions.csv`, `sdkconfig.defaults*`, `main/aera660_gdl90_uart.c`
  (GDL90 framer/CRC), `main/aera660_wifi.c` (SoftAP + http), `components/wxstore/`.
- `connext-emulator/docs/FINDINGS-translator.md` — TTGO T8 hardware findings.
- Stratux Go sources (sibling repo): `main/pong.go`, `gen_gdl90.go`, `traffic.go`,
  `network.go`, `uatparse/`.
