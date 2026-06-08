# stratux-esp32

A port of [Stratux](https://github.com/stratux/stratux) to the ESP32 (LilyGO
**TTGO T8**), fed by an integrated **uAvionix Pong** ADS-B radio and emitting
**GDL90 over WiFi** to EFB apps (ForeFlight, Garmin Pilot, FltPlan Go, …).

Because the Pong delivers already-demodulated 1090ES + 978 UAT frames over serial,
the ESP32 needs no SDR/DSP — just a line parser, frame decoders, a traffic table,
and a GDL90 encoder over WiFi.

**Status:** scaffold / pre-M0. See **[AGENTS.md](AGENTS.md)** for the build/flash
commands, architecture, hardware/pin plan, repository layout, conventions, and
milestone roadmap.
