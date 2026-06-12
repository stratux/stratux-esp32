#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// WiFi SoftAP + GDL90 UDP delivery. AP defaults: SSID "stratux"
// (from g_settings), gateway/AP IP 192.168.10.1, GDL90 on UDP :4000 — so
// existing EFB setups "just work."

// Bring up WiFi from g_settings and open the :4000 UDP sender socket. Always
// starts the SoftAP; when g_settings.sta_en is set, also joins sta_ssid in
// APSTA mode (the AP follows the STA's channel once associated — wifi_chan
// only governs operation before/without a join).
void net_wifi_start(void);

// STA-side state for /getStatus and the serial $WIFI GET. Dotted quads are
// "0.0.0.0" until DHCP completes.
typedef struct {
    bool enabled;     // g_settings.sta_en
    bool connected;   // associated and holding a DHCP address
    char ip[16];
    char gw[16];
    char dns[16];
} net_sta_status_t;
void net_sta_status(net_sta_status_t *out);

// Parse a comma-separated IPv4 list (<=4 entries) into the live static GDL90
// target cache; empty string clears it. Validates the whole list before
// committing. Returns the count parsed, or -1 on a bad token / too many.
int net_set_static_dest(const char *csv);

// Count of currently-associated SoftAP stations (for /getStatus).
int net_client_count(void);

// Count of active EFB DHCP leases (the actual GDL90 unicast destinations).
int net_lease_count(void);

// Send one GDL90 datagram, unicast, to every EFB client on UDP :4000.
//
// Delivery mirrors Stratux (network.go): the SoftAP's DHCP server is the lease
// source — clients are tracked via IP_EVENT_AP_STAIPASSIGNED /
// WIFI_EVENT_AP_STADISCONNECTED — and each datagram is unicast to every lease.
// We do NOT broadcast: 802.11 sends broadcast/multicast at the lowest basic
// rate, buffered until DTIM, so power-saving EFB tablets drop them.
void net_gdl90_send(const uint8_t *datagram, size_t len);
