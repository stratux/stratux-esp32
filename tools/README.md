# tools/

Dev/bring-up helpers. To be adapted from `connext-emulator/esp32/`.
Placeholders / planned:

- **flash/monitor** — wrappers around `idf.py -b 115200 flash monitor` (the
  USB-serial bridge is unreliable above 115200 — see AGENTS.md "Gotchas").
- **storage-image** — pack `www/` into the FAT `storage` partition and flash it
  (adapt from connext packaging scripts).
- **stratux-bridge.py** — host-side bridge/replay for feeding canned Pong lines
  to a dev board over serial (adapt from connext `stratux-bridge.py`).
- **set-time.py** — push a manual `TIME <epoch>` to the device before GPS exists
  (adapt from connext `set-time.py`; relevant to M0/M3 time source).
