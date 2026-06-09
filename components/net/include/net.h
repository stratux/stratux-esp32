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

// Send one GDL90 datagram, unicast, to every EFB client on UDP :4000.
//
// Delivery mirrors Stratux (network.go): the SoftAP's DHCP server is the lease
// source — clients are tracked via IP_EVENT_AP_STAIPASSIGNED /
// WIFI_EVENT_AP_STADISCONNECTED — and each datagram is unicast to every lease.
// We do NOT broadcast: 802.11 sends broadcast/multicast at the lowest basic
// rate, buffered until DTIM, so power-saving EFB tablets drop them.
void net_gdl90_send(const uint8_t *datagram, size_t len);
