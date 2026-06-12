# TESTING.md — required verification before closing M1

M1 is code-complete: host unit tests pass (`tools/test_modes.c`,
`tools/test_uat.c`), and the 1090ES decode path is validated against the real
capture `ponglog-07062025.log` (63,268 `*` lines → 31,693 decoded, 10,052
resolved CPR positions via `tools/replay_modes.c`). Two items remain. Item 1 is
**deferred**; item 2 is the M1 acceptance test, instructions below.

## 1. UAT downlink validation against real off-air data — DEFERRED

The bundled capture contains zero `-` (UAT downlink) lines — no 978-equipped
aircraft were heard during the recording — so the UAT downlink decoder
(`components/uat/`) has only been exercised by the synthetic vectors in
`tools/test_uat.c`, never real off-air frames.

To close later, one of:

- Record a new capture (`tools/pong_capture.py`) somewhere with 978 traffic
  (US, GA-heavy airspace below 18,000 ft) and confirm `-` lines decode via
  `tools/replay_uat.c` (build line in its header comment), then end-to-end per
  the procedure below.
- Or splice known-good dump978 sample frames into a `.ponglog` as `-` lines
  with plausible `;rs=…;ss=…;` suffixes and replay them.

Pass criteria when un-deferred: `replay_uat` shows decoded MDBs with positions
and no `bad length`/`bad hex` spikes; on-device, UAT targets appear as 0x14
traffic reports (address types 0–5 per the DO-282 qualifier).

## 2. End-to-end traffic test (capture → board → GDL90 → `gdl90_listen.py`)

Verifies the full on-device pipeline: console replay → line parser → Mode-S
decode + CPR → traffic table → GDL90 0x14/0x00/0xCC framing → per-lease UDP
unicast on :4000. One Mac and one USB cable suffice: the capture streams in
over USB-serial while the Mac's WiFi joins the board's SoftAP to receive GDL90.

### Setup

1. **Build the firmware in console-replay mode.** The USB bridge can't do
   3 Mbaud, so the dev harness feeds Pong lines in on the console UART0 at
   115200 (`CONFIG_PONG_SOURCE_CONSOLE`, see `components/pong/Kconfig`):

   ```bash
   . $IDF_PATH/export.sh
   echo 'CONFIG_PONG_SOURCE_CONSOLE=y' >> sdkconfig   # or idf.py menuconfig → "Pong input source"
   idf.py -B build-wifi -D RADIO=wifi build
   ```

   Note: appending to the generated `sdkconfig` works; editing
   `sdkconfig.defaults*` does NOT retroactively apply (AGENTS.md).

2. **Flash at 115200** (the TTGO T8 bridge is unreliable above that):

   ```bash
   idf.py -B build-wifi -p /dev/cu.usbserial-XXXX -b 115200 flash
   ```

   Do **not** start `idf.py monitor` — `pong_replay.py` owns the serial port
   and prints the board's log output itself (`<`-prefixed lines).

3. **Reconstruct capture timestamps** (one-time; the legacy log is
   un-timestamped, timing is RANSAC-fitted from FIS-B uplink time fields):

   ```bash
   python3 tools/pong_reconstruct.py ponglog-07062025.log -o /tmp/cap.ponglog
   ```

4. **Join the board's WiFi.** Connect the Mac's WiFi to the open `stratux` AP.
   Getting a DHCP lease (a `192.168.10.x` address) is itself part of the test —
   the firmware unicasts GDL90 only to leases, never broadcasts. The board logs
   `EFB client lease 192.168.10.x` when you join.

### Run

In terminal A, start the listener (pure stdlib, no deps):

```bash
python3 tools/gdl90_listen.py --seconds 120
```

In terminal B, stream the capture in real time (needs `pip install pyserial`):

```bash
python3 tools/pong_replay.py /tmp/cap.ponglog --rate 1.0 -p /dev/cu.usbserial-XXXX
```

Keep `--rate` at 1.0 (or ≤ ~3): the CPR store requires an even/odd pair within
10 s of *arrival*, so grossly slowed replay breaks position resolution, and
rates past ~3–4× saturate the 115200 link. The replay tool pulses the board
into run mode on start, so each run begins with a fresh boot log.

### Pass criteria

- `gdl90_listen.py` reports **zero CRC failures** and **zero deframe errors**.
- `Heartbeat` (0x00) and `Stratux-HB` (0xCC) each at ~1.0/s. Heartbeat shows
  `UTC_OK=0 ts=0` — correct pre-M3 (no time source; the firmware must not lie).
- `Traffic` (0x14) messages appear within ~30 s of replay start and the rate
  climbs as CPR pairs resolve (one 0x14 per positioned target per second; the
  capture peaks at tens of concurrent aircraft). Use `--raw` to eyeball frames:
  payload starts `14`, byte 1 low nibble = address type, bytes 2–4 = ICAO.
- The board's 5 s diag line (read back by `pong_replay.py`) corroborates:
  `emit: es_msgs=N table=T positioned=P assoc=1 leases=1` with `es_msgs`
  climbing, `positioned ≥ 1`, and `leases=1` (you).
- After replay ends, traffic rates decay: stale positions demote after 15 s,
  entries age out after 60 s — 0x14 output should cease, heartbeats continue.

### Gold check (optional but definitive)

With the replay looping (`--loop`), open ForeFlight / Garmin Pilot / FltPlan Go
on a tablet joined to the `stratux` AP: the app should show the device
connected (heartbeat) and paint moving traffic targets with altitudes,
N-number callsigns, and climb/descent arrows. This is the M1 exit criterion as
an EFB user sees it.

### Cleanup

Restore the production frame source before any radio-attached build:

```bash
# remove CONFIG_PONG_SOURCE_CONSOLE=y from sdkconfig (or delete sdkconfig
# to re-apply defaults), then rebuild
idf.py -B build-wifi -D RADIO=wifi build
```

### Recording results

When the test passes, note the date, firmware commit, peak `positioned` count,
and listener message-rate summary here (and flip the AGENTS.md status line —
it still reads "scaffold / pre-M0").

| Date | Commit | Result |
|---|---|---|
| 2026-06-11 | 8752f14 | **PASS** — 120 s listen window: Heartbeat 117 (0.99/s, `UTC_OK=0 ts=0`), Stratux-HB 117 (0.99/s), Traffic 1,884 (15.88/s ≈ 16 concurrent positioned targets), zero CRC/deframe failures. Replay: `ponglog-07062025.ponglog` at `--rate 1.0` over console UART0. |

## 3. M2 web UI verification

Host-side already verified: `tools/replay_uplink.c` over `ponglog-07062025.log`
(1,564 `+` lines → 1,049 walked frames, 7,408 FIS-B APDUs across 13 products,
counts byte-identical to a `fisb_time.py`-based Python cross-check; the 515
rejects are 514 short fragments + 1 odd-length line).

On-device procedure — same replay setup as test 2, then in a browser at
`http://192.168.10.1/`:

1. **Status strip** updates at 1 Hz: ES/UAT counters climb, uplink count grows,
   version shows `0.2.0-m2`. **Pong link**: shows "up" while replay lines flow
   (no `--heartbeat` flag needed — any line counts as liveness; a real Pong
   heartbeats `.` only after 5 s of idle), and flips to "down" within ~8 s of
   stopping the replay.
2. **Traffic table** populates live over the WebSocket within ~30 s; rows show
   ICAO, tail, source (ES), positions with ✱ when extrapolated; rows age out
   after replay stops.
3. **UAT products table** fills (this capture: NEXRAD Regional ~3.0k, Text
   ~1.2k, NOTAM ~1.1k, Cloud Tops, Turbulence, Icing, Lightning, …).
4. **Raw Pong feed** (collapsible) shows live lines when expanded.
5. **Settings round-trip**: toggle "1090ES decode" off → Save → traffic stops
   without reboot; change WiFi channel → Save → "rebooting" → rejoin AP →
   the form shows the persisted channel.
6. **Regression**: `gdl90_listen.py --seconds 60` — heartbeats + traffic, zero
   CRC failures; heartbeat bytes 5–6 (message counts) now non-zero (`--raw`).

| Date | Commit | Result |
|---|---|---|
| 2026-06-11 | cb35348 | PASS — all 6 checks (status strip + Pong up/down, live traffic WS + ✱ + age-out, UAT products NEXRAD ~3.0k/Text ~1.2k/NOTAM ~1.1k, raw Pong feed, settings round-trip, gdl90_listen 60 s zero CRC fails). Replay of `ponglog-07062025.ponglog` over UART0. |

## 4. WiFi client (STA) mode + serial config channel — UNVERIFIED on-device

Both build variants compile (production and `CONFIG_PONG_SOURCE_CONSOLE`);
the checks below need real hardware + a real network.

1. **No-reset serial get**: `tools/wifi_config.py -p <console> get` →
   `$OK sta_en=0 ... state=disabled` and `$OK dest=` — and NO boot banner in
   the output (the tool must not reset the board on connect).
2. **Join a network**: `wifi_config.py -p <port> set --enable --ssid <net>
   --pass <pw>` → `$OK saved (reboot to apply)`; `reboot`; then `get` →
   `state=connected ip=... gw=... dns=...`. The `stratux` AP must still beacon
   and an EFB on it must still receive traffic.
3. **Static GDL90 target**: `set --dest <LAN host>` (applies live, no reboot);
   `gdl90_listen.py` on that host sees ~1/s heartbeats with zero CRC failures
   *without* joining the SoftAP.
4. **Web UI**: settings form round-trips the four new fields; changing
   sta_en/sta_ssid/sta_pass replies `reboot:true`; a bad IP in "GDL90 targets"
   → 400 and nothing saved; status strip shows `STA <ip> (gw .., dns ..)`.
5. **Resilience**: power off the joined AP → backoff reconnect logs (1→30 s
   cap), SoftAP/EFB delivery unaffected; power restore → reconnects.
6. **Negative serial**: `$WIFI SET pass=short` → `$ERR`; `$BOGUS` → `$ERR
   unknown command`; a quoted SSID with spaces round-trips through `get`.
7. **Replay-build coexistence**: with `CONFIG_PONG_SOURCE_CONSOLE=y`,
   `wifi_config.py get` works on the same port as `pong_replay.py` ('$' lines
   route through pong's classifier; capture lines never start with '$').

| Date | Commit | Result |
|---|---|---|
