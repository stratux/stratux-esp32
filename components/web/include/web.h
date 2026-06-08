#pragma once

// Start esp_http_server over the SoftAP. Endpoint shapes
// mirror Pi Stratux so concepts/tools carry over:
//   GET  /getStatus    JSON snapshot (version, Pong link, msg counts, clients)
//   GET  /getSettings  + POST /setSettings   (NVS via settings.{c,h})  [M2]
//   GET  /getTraffic   + WS /traffic         live traffic               [M2]
//   WS   /status                                                         [M2]
// Static www/* assets are served from the FAT `storage` partition (M2).
void web_start(void);
