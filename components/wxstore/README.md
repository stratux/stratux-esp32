# wxstore (M5 — not yet implemented)

Tiered FIS-B weather store. Deferred to **M5b** (see the milestone roadmap in
[AGENTS.md](../../AGENTS.md)). To be ported from
`connext-emulator/esp32/components/wxstore/`.

Tiers:
- **Tier 1 — PSRAM** (4 MB mapped): live index, per-product TTL, dedup.
- **Tier 2 — himem** (upper 4 MB, 32 KB banks): rolling archive.
- **Tier 3 — microSD** (SDSPI/SDMMC, separate from flash): durable write-through.

Requires the **WROVER/8 MB-PSRAM overlay** (ported from
`connext-emulator/esp32/sdkconfig.wrover`). The on-flash FAT `storage` partition
holds the **web UI assets**, NOT weather — do not conflate them.
