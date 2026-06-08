#pragma once
#include <stddef.h>
#include <stdint.h>

// WiFi SoftAP + GDL90 UDP delivery. AP defaults: SSID "stratux"
// (from g_settings), gateway/AP IP 192.168.10.1, GDL90 on UDP :4000 — so
// existing EFB setups "just work."

// Bring up the SoftAP from g_settings and open the :4000 UDP sender socket.
void net_wifi_softap_start(void);

// Count of currently-associated SoftAP stations (for /getStatus).
int net_client_count(void);

// Send one GDL90 datagram to the EFB clients on UDP :4000.
//
// Delivery strategy is an OPEN question (see AGENTS.md): there is no
// dnsmasq lease file on the ESP32. M0 uses a single subnet broadcast to
// 192.168.10.255; M2 should enumerate associated stations (track
// IP_EVENT_AP_STAIPASSIGNED) and unicast each. Validate which ForeFlight /
// Garmin Pilot accept before committing.
void net_gdl90_send(const uint8_t *datagram, size_t len);
