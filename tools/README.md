# tools/

Dev/bring-up helpers.

## Pong capture / replay (host-side, Python)

Timestamped-capture toolchain for recording and replaying Pong serial streams.
Capture format (`pong_capture_format.py`): one record per line,
`<epoch_ms> <raw Pong line>`, where `epoch_ms` is UTC milliseconds since the
Unix epoch. `#` lines are comments. Replay/capture need **pyserial**
(`pip install pyserial`); reconstruct is pure-stdlib.

- **`pong_capture.py`** — read a live Pong over serial and write a timestamped
  capture: `pong_capture.py -p <port> -o out.ponglog [--echo]`. Default port is
  the TTGO bridge in AGENTS.md; baud defaults to 3,000,000.
- **`pong_replay.py`** — the main debugging harness: inject a capture into the
  board's Pong UART honoring inter-line timing scaled by `--rate` (`1.0`=real
  time, `>1` faster, `<1` slower) **while concurrently reading the board's
  serial output back** and printing both streams interleaved (`>` = injected,
  `<` = read back, each with a relative timestamp).
  `pong_replay.py out.ponglog --rate 1.0 -p <pong-uart> [--monitor-port <console>
  --monitor-baud 115200] [--loop --max-gap S] [--echo] [--monitor-out log]`.
  Monitoring defaults to the inject port (full-duplex); point `--monitor-port`
  at the ESP32 console (UART0, 115200) to watch firmware logs. `--no-monitor`
  disables reading; `--dry-run` prints injected lines without opening serial.
- **`pong_reconstruct.py`** — synthesize timestamps for a legacy
  un-timestamped `ponglog-*.log`. UAT uplink (FIS-B) frames embed UTC time
  fields; `fisb_time.py` decodes them and RANSAC-fits line-number → wall-clock,
  then lines are evenly spaced at the inferred rate (per-line jitter isn't
  recoverable). For `ponglog-07062025.log` this infers ~48.5 lines/s over
  ~22.3 min starting 2025-07-06 19:57:53 UTC. Override with `--rate` / `--date`
  / `--start`.
- **`fisb_time.py`** — FIS-B uplink time decoder + fit (ported/verified against
  Stratux `uatparse/uatparse.go`); imported by reconstruct.
- **`gdl90_listen.py`** — validate the firmware's GDL90 output: join the device's
  `stratux` AP (so it leases you an address and unicasts to you), then
  `gdl90_listen.py [--seconds N] [--raw]`. Deframes, verifies the GDL90-variant
  CRC (`123456789 -> 0xBEEF`), decodes heartbeat fields, and reports per-message
  rates. The M0 check: ~1/s `Heartbeat` with `UTC_OK=0 ts=0` and no CRC failures.
  Pure stdlib.

### Firmware dev mode — replay over the single USB cable

The onboard USB-serial bridge can't do 3 Mbaud (AGENTS.md: flashing is flaky
above 115200), so don't inject the real-radio baud over USB. Instead build the
firmware to read Pong frames from the **console UART0** at 115200:

```bash
idf.py menuconfig        # Pong input source -> "Replay over console USB (UART0)"
# or: echo 'CONFIG_PONG_SOURCE_CONSOLE=y' >> sdkconfig
idf.py -B build-wifi build flash       # then run the harness below (NOT idf.py monitor)
```

In this mode the firmware ingests injected lines on UART0 RX and still logs on
UART0 TX, so one cable does both. The ~3 KB/s real-time stream fits 115200 with
headroom; only `--rate` past ~3-4x would saturate it. Production builds leave
the source on the real radio (UART2 @ 3 Mbaud) — the default.

`pong_replay.py` deasserts DTR/RTS and pulses the board's auto-reset into run
mode on start, so it never falls into the silent ROM download mode and you
capture a fresh boot log each run (the ESP32's USB bridge wires DTR/RTS to
GPIO0/EN — opening a port carelessly otherwise resets it into the bootloader
and you see *no output*). Use `--no-reset` to leave a running board alone, and
`--boot-wait SEC` to tune the post-reset settle before injection starts.

Example end-to-end: reconstruct the bundled log and stream it to a board in
real time over the single USB cable, watching firmware logs interleaved:

```bash
python3 tools/pong_reconstruct.py ponglog-07062025.log -o /tmp/cap.ponglog
python3 tools/pong_replay.py /tmp/cap.ponglog --rate 1.0 \
    -p /dev/cu.usbserial-5C8C0111211        # replay defaults to 115200
```

## WiFi / device config (host-side, Python)

- **`wifi_config.py`** — configure WiFi client (STA) mode and the static GDL90
  target IPs over the USB console, via the firmware's `$` command channel
  (`components/console_cmd/`, see AGENTS.md):
  `wifi_config.py -p <console> get` / `set --enable --ssid NET --pass PW`
  / `set --dest ip,ip` / `reboot`. Opens the port with DTR/RTS deasserted so
  the board is **not** reset; replies (`$OK`/`$ERR`) are filtered out of the
  interleaved log stream. WiFi changes apply on reboot; `--dest` applies live.
  Needs pyserial. Works in both production and console-replay builds.

## Planned (to be adapted from `connext-emulator/esp32/`)

- **flash/monitor** — wrappers around `idf.py -b 115200 flash monitor` (the
  USB-serial bridge is unreliable above 115200 — see AGENTS.md "Gotchas").
- **storage-image** — pack `www/` into the FAT `storage` partition and flash it
  (adapt from connext packaging scripts).
- **stratux-bridge.py** — host-side bridge/replay for feeding canned Pong lines
  to a dev board over serial (adapt from connext `stratux-bridge.py`).
- **set-time.py** — push a manual `TIME <epoch>` to the device before GPS exists
  (adapt from connext `set-time.py`; relevant to M0/M3 time source).
