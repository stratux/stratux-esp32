#include "net.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_timer.h"
#include "esp_mac.h"
#include "lwip/sockets.h"
#include "lwip/etharp.h"
#include "settings.h"

static const char *TAG = "net";

#define GDL90_UDP_PORT   4000
#define MAX_CLIENTS      8        // SoftAP max_connection is 4; headroom for churn
#define MAX_STATIC_DEST  4        // user-configured GDL90 unicast targets
#define MAX_DEST         (MAX_CLIENTS + MAX_STATIC_DEST)

// STA reconnect backoff: 1 s doubling to a 30 s cap. Connecting is fully
// async — boot never waits on the join.
#define STA_RETRY_MIN_S  1
#define STA_RETRY_MAX_S  30

// Deauth-nudge: if a station associates but never re-runs DHCP (common after an
// ESP32 reboot — the client keeps its cached IP, so IP_EVENT_AP_STAIPASSIGNED
// never fires and we have no address to unicast GDL90 to), kick it once so it
// re-associates and re-DHCPs. Addresses the root cause without persisting leases.
#define NUDGE_GRACE_MS   6000     // let a fresh client DHCP on its own first
#define NUDGE_REARM_MS   30000    // a reconnect this long after a kick is "fresh"
#define NUDGE_MAX_KICKS  3        // bound attempts so a non-DHCP client can't loop
#define NUDGE_PERIOD_US  (2 * 1000 * 1000)

static int               s_udp_sock = -1;
static SemaphoreHandle_t s_lease_mux;
static esp_netif_t      *s_ap_netif;   // SoftAP handle, for DHCP lease lookups
static esp_timer_handle_t s_nudge_timer;

// STA (WiFi client) state. s_sta_ip doubles as the "connected" flag.
static esp_netif_t       *s_sta_netif;
static volatile uint32_t  s_sta_ip;        // network byte order; 0 = not connected
static volatile uint32_t  s_sta_gw;
static esp_timer_handle_t s_sta_retry_timer;
static int                s_sta_retry_s = STA_RETRY_MIN_S;

// Static GDL90 unicast targets (g_settings.gdl90_dest), guarded by s_lease_mux.
static uint32_t s_static_dest[MAX_STATIC_DEST];
static int      s_static_dest_n;

// EFB "lease table": one entry per associated station the SoftAP's DHCP server
// has handed an address. This is the ESP32 equivalent of Stratux's
// getDHCPLeases() (network.go): the SoftAP's own DHCP server is our lease
// source, populated from IP_EVENT_AP_STAIPASSIGNED and pruned on
// WIFI_EVENT_AP_STADISCONNECTED. GDL90 is then *unicast* to each lease, exactly
// as Stratux does — never broadcast. Broadcast/multicast frames leave the AP at
// the lowest basic rate and are buffered until DTIM, so power-saving EFB
// tablets routinely drop them; per-client unicast is the reliable path.
typedef struct {
    bool     used;
    uint8_t  mac[6];
    uint32_t ip;      // network byte order, ready for sockaddr_in.sin_addr
} lease_t;

static lease_t s_leases[MAX_CLIENTS];

// Association tracking for the deauth-nudge (separate from the lease cache: a
// station can be associated without a lease — that is exactly the case we fix).
typedef struct {
    bool     used;
    bool     associated;
    uint8_t  mac[6];
    uint16_t aid;
    int64_t  since_ms;     // association / last-rearm time (grace window start)
    int64_t  last_kick_ms;
    int      kicks;
} assoc_t;

static assoc_t s_assoc[MAX_CLIENTS];

// Mark a station associated (from WIFI_EVENT_AP_STACONNECTED). Re-arms the grace
// window; clears the kick counter only if this looks like a genuine fresh join
// (not the re-association our own deauth just triggered).
static void assoc_connected(const uint8_t mac[6], uint16_t aid, int64_t now_ms)
{
    xSemaphoreTake(s_lease_mux, portMAX_DELAY);
    int slot = -1;
    for (int i = 0; i < MAX_CLIENTS; i++) {
        if (s_assoc[i].used && memcmp(s_assoc[i].mac, mac, 6) == 0) { slot = i; break; }
        if (!s_assoc[i].used && slot < 0) slot = i;
    }
    if (slot >= 0) {
        assoc_t *a = &s_assoc[slot];
        bool fresh = !a->used || (now_ms - a->last_kick_ms) > NUDGE_REARM_MS;
        a->used = true;
        a->associated = true;
        memcpy(a->mac, mac, 6);
        a->aid = aid;
        a->since_ms = now_ms;
        if (fresh) a->kicks = 0;
    }
    xSemaphoreGive(s_lease_mux);
}

static void assoc_disconnected(const uint8_t mac[6])
{
    xSemaphoreTake(s_lease_mux, portMAX_DELAY);
    for (int i = 0; i < MAX_CLIENTS; i++) {
        if (s_assoc[i].used && memcmp(s_assoc[i].mac, mac, 6) == 0) {
            // Keep the entry (kick bookkeeping must survive the brief
            // disconnect our own deauth causes); just mark it not associated.
            s_assoc[i].associated = false;
        }
    }
    xSemaphoreGive(s_lease_mux);
}

// A successful lease means the client is reachable — stop nudging it.
static void assoc_resolved(const uint8_t mac[6])
{
    xSemaphoreTake(s_lease_mux, portMAX_DELAY);
    for (int i = 0; i < MAX_CLIENTS; i++) {
        if (s_assoc[i].used && memcmp(s_assoc[i].mac, mac, 6) == 0) {
            s_assoc[i].kicks = 0;
        }
    }
    xSemaphoreGive(s_lease_mux);
}

static void lease_upsert(const uint8_t mac[6], uint32_t ip)
{
    xSemaphoreTake(s_lease_mux, portMAX_DELAY);
    int slot = -1;
    for (int i = 0; i < MAX_CLIENTS; i++) {
        if (s_leases[i].used && memcmp(s_leases[i].mac, mac, 6) == 0) {
            s_leases[i].ip = ip;               // renewal / re-assignment
            xSemaphoreGive(s_lease_mux);
            return;
        }
        if (!s_leases[i].used && slot < 0) slot = i;
    }
    if (slot >= 0) {
        s_leases[slot].used = true;
        memcpy(s_leases[slot].mac, mac, 6);
        s_leases[slot].ip = ip;
    } else {
        ESP_LOGW(TAG, "lease table full (%d); dropping new client", MAX_CLIENTS);
    }
    xSemaphoreGive(s_lease_mux);
}

static void lease_remove(const uint8_t mac[6])
{
    xSemaphoreTake(s_lease_mux, portMAX_DELAY);
    for (int i = 0; i < MAX_CLIENTS; i++) {
        if (s_leases[i].used && memcmp(s_leases[i].mac, mac, 6) == 0) {
            s_leases[i].used = false;
        }
    }
    xSemaphoreGive(s_lease_mux);
}

static void sta_retry_cb(void *arg)
{
    (void)arg;
    esp_wifi_connect();
}

static void net_event_handler(void *arg, esp_event_base_t base,
                              int32_t id, void *data)
{
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
        return;
    }
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        wifi_event_sta_disconnected_t *e = (wifi_event_sta_disconnected_t *)data;
        bool was_up = s_sta_ip != 0;
        s_sta_ip = 0;
        s_sta_gw = 0;
        if (was_up)
            ESP_LOGW(TAG, "STA disconnected from %s (reason %d)",
                     g_settings.sta_ssid, e->reason);
        if (s_sta_retry_timer) {
            esp_timer_start_once(s_sta_retry_timer,
                                 (uint64_t)s_sta_retry_s * 1000 * 1000);
            ESP_LOGI(TAG, "STA retry in %ds", s_sta_retry_s);
            if (s_sta_retry_s < STA_RETRY_MAX_S) s_sta_retry_s *= 2;
            if (s_sta_retry_s > STA_RETRY_MAX_S) s_sta_retry_s = STA_RETRY_MAX_S;
        }
        return;
    }
    if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *e = (ip_event_got_ip_t *)data;
        s_sta_ip = e->ip_info.ip.addr;
        s_sta_gw = e->ip_info.gw.addr;
        s_sta_retry_s = STA_RETRY_MIN_S;
        ESP_LOGI(TAG, "STA joined %s: ip=" IPSTR " gw=" IPSTR,
                 g_settings.sta_ssid, IP2STR(&e->ip_info.ip), IP2STR(&e->ip_info.gw));
        return;
    }

    if (base == IP_EVENT && id == IP_EVENT_AP_STAIPASSIGNED) {
        ip_event_ap_staipassigned_t *e = (ip_event_ap_staipassigned_t *)data;
        lease_upsert(e->mac, e->ip.addr);
        assoc_resolved(e->mac);
        ESP_LOGI(TAG, "EFB client lease " IPSTR, IP2STR(&e->ip));
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t *e = (wifi_event_ap_staconnected_t *)data;
        assoc_connected(e->mac, e->aid, esp_timer_get_time() / 1000);
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t *e = (wifi_event_ap_stadisconnected_t *)data;
        lease_remove(e->mac);
        assoc_disconnected(e->mac);
        ESP_LOGI(TAG, "EFB client disconnected; lease dropped");
    }
}

// True if the DHCP server (or our event cache) currently knows an IP for this
// MAC — i.e. the client has completed DHCP and is reachable for unicast.
static bool mac_has_ip(const uint8_t mac[6])
{
    xSemaphoreTake(s_lease_mux, portMAX_DELAY);
    for (int i = 0; i < MAX_CLIENTS; i++) {
        if (s_leases[i].used && s_leases[i].ip &&
            memcmp(s_leases[i].mac, mac, 6) == 0) {
            xSemaphoreGive(s_lease_mux);
            return true;
        }
    }
    xSemaphoreGive(s_lease_mux);

    if (s_ap_netif) {
        esp_netif_pair_mac_ip_t pair;
        memset(&pair, 0, sizeof(pair));
        memcpy(pair.mac, mac, 6);
        if (esp_netif_dhcps_get_clients_by_mac(s_ap_netif, 1, &pair) == ESP_OK &&
            pair.ip.addr != 0) {
            return true;
        }
    }
    return false;
}

// MAC -> IP via the lwIP ARP table. A station that kept its cached IP across
// an ESP32 reboot (so the DHCP server has no binding for it) still populates
// ARP the moment it sends any IP traffic — that IP is as good as a lease.
// Read-only scan without the tcpip lock: the table is static storage, and the
// worst race is one stale/garbled adoption that the next real DHCP event or
// nudge pass corrects.
static bool arp_ip_for_mac(const uint8_t mac[6], uint32_t *ip_out)
{
    ip4_addr_t *ip;
    struct netif *nif;
    struct eth_addr *eth;
    for (size_t i = 0; i < ARP_TABLE_SIZE; i++) {
        if (etharp_get_entry(i, &ip, &nif, &eth) &&
            memcmp(eth->addr, mac, 6) == 0) {
            *ip_out = ip->addr;
            return true;
        }
    }
    return false;
}

// Periodic: for any station associated past the grace window with no
// resolvable IP, first try to ADOPT its address from the ARP table (a client
// reusing a cached lease across our reboot is reachable as-is — deauthing it
// just causes a long, pointless outage); only kick stations ARP has never
// seen, so they re-associate and re-run DHCP.
static void nudge_timer_cb(void *arg)
{
    (void)arg;
    int64_t now = esp_timer_get_time() / 1000;

    // Snapshot candidates under the lock, act (deauth) outside it.
    struct { uint8_t mac[6]; uint16_t aid; int slot; } cand[MAX_CLIENTS];
    int nc = 0;

    xSemaphoreTake(s_lease_mux, portMAX_DELAY);
    for (int i = 0; i < MAX_CLIENTS; i++) {
        assoc_t *a = &s_assoc[i];
        if (a->used && a->associated &&
            a->kicks < NUDGE_MAX_KICKS &&
            (now - a->since_ms) >= NUDGE_GRACE_MS) {
            memcpy(cand[nc].mac, a->mac, 6);
            cand[nc].aid = a->aid;
            cand[nc].slot = i;
            nc++;
        }
    }
    xSemaphoreGive(s_lease_mux);

    for (int i = 0; i < nc; i++) {
        if (mac_has_ip(cand[i].mac))
            continue;   // resolved on its own during the grace window

        uint32_t ip;
        if (arp_ip_for_mac(cand[i].mac, &ip)) {
            ESP_LOGI(TAG, "no DHCP lease for " MACSTR " but ARP knows "
                          IPSTR "; adopting as lease", MAC2STR(cand[i].mac),
                     IP2STR((ip4_addr_t *)&ip));
            lease_upsert(cand[i].mac, ip);
            assoc_resolved(cand[i].mac);
            continue;
        }

        ESP_LOGW(TAG, "no DHCP lease for " MACSTR " (aid=%u) after %dms; "
                      "deauth to force re-DHCP", MAC2STR(cand[i].mac),
                 cand[i].aid, NUDGE_GRACE_MS);
        esp_wifi_deauth_sta(cand[i].aid);

        xSemaphoreTake(s_lease_mux, portMAX_DELAY);
        assoc_t *a = &s_assoc[cand[i].slot];
        if (a->used && memcmp(a->mac, cand[i].mac, 6) == 0) {
            a->kicks++;
            a->last_kick_ms = now;
            a->since_ms = now;   // re-arm grace for the next check
        }
        xSemaphoreGive(s_lease_mux);
    }
}

void net_wifi_start(void)
{
    bool sta = g_settings.sta_en && g_settings.sta_ssid[0];

    s_lease_mux = xSemaphoreCreateMutex();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_t *ap = esp_netif_create_default_wifi_ap();
    s_ap_netif = ap;
    if (sta) s_sta_netif = esp_netif_create_default_wifi_sta();  // DHCP client by default

    // Pin the AP to 192.168.10.1/24 so the gateway matches Stratux defaults and
    // DHCP leases land on the subnet EFBs expect. Stop the default DHCP server,
    // set the static IP, then restart it so clients get 192.168.10.x.
    esp_netif_ip_info_t ipi;
    memset(&ipi, 0, sizeof(ipi));
    IP4_ADDR(&ipi.ip,      192, 168, 10, 1);
    IP4_ADDR(&ipi.gw,      192, 168, 10, 1);
    IP4_ADDR(&ipi.netmask, 255, 255, 255, 0);
    esp_err_t derr = esp_netif_dhcps_stop(ap);
    if (derr != ESP_OK && derr != ESP_ERR_ESP_NETIF_DHCP_ALREADY_STOPPED) {
        ESP_ERROR_CHECK(derr);
    }
    ESP_ERROR_CHECK(esp_netif_set_ip_info(ap, &ipi));
    ESP_ERROR_CHECK(esp_netif_dhcps_start(ap));

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    // Track DHCP assignments / disconnects to maintain the lease table. Register
    // before esp_wifi_start() so no early client event is missed.
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_AP_STAIPASSIGNED, net_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, WIFI_EVENT_AP_STACONNECTED, net_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, WIFI_EVENT_AP_STADISCONNECTED, net_event_handler, NULL, NULL));
    if (sta) {
        ESP_ERROR_CHECK(esp_event_handler_instance_register(
            WIFI_EVENT, WIFI_EVENT_STA_START, net_event_handler, NULL, NULL));
        ESP_ERROR_CHECK(esp_event_handler_instance_register(
            WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED, net_event_handler, NULL, NULL));
        ESP_ERROR_CHECK(esp_event_handler_instance_register(
            IP_EVENT, IP_EVENT_STA_GOT_IP, net_event_handler, NULL, NULL));

        const esp_timer_create_args_t retry_args = {
            .callback = sta_retry_cb,
            .name = "sta_retry",
        };
        ESP_ERROR_CHECK(esp_timer_create(&retry_args, &s_sta_retry_timer));
    }

    wifi_config_t wc = { 0 };
    strncpy((char *)wc.ap.ssid, g_settings.wifi_ssid, sizeof(wc.ap.ssid));
    wc.ap.ssid_len = strlen(g_settings.wifi_ssid);
    wc.ap.channel  = g_settings.wifi_chan ? g_settings.wifi_chan : 1;
    wc.ap.max_connection = 4;
    if (g_settings.wifi_pass[0]) {
        strncpy((char *)wc.ap.password, g_settings.wifi_pass, sizeof(wc.ap.password));
        wc.ap.authmode = WIFI_AUTH_WPA2_PSK;
    } else {
        wc.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(sta ? WIFI_MODE_APSTA : WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wc));
    if (sta) {
        wifi_config_t sc = { 0 };
        strncpy((char *)sc.sta.ssid, g_settings.sta_ssid, sizeof(sc.sta.ssid));
        if (g_settings.sta_pass[0]) {
            strncpy((char *)sc.sta.password, g_settings.sta_pass,
                    sizeof(sc.sta.password));
            sc.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
        } else {
            sc.sta.threshold.authmode = WIFI_AUTH_OPEN;
        }
        sc.sta.scan_method = WIFI_ALL_CHANNEL_SCAN;
        ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &sc));
        // In APSTA the radio is single-channel: once the STA associates, the
        // SoftAP moves to the STA network's channel and wifi_chan is moot.
        ESP_LOGI(TAG, "STA client enabled (SSID=%s); AP will follow the STA "
                      "channel once joined", g_settings.sta_ssid);
    }
    ESP_ERROR_CHECK(esp_wifi_start());

    // GDL90 UDP sender. Datagrams are unicast per lease in net_gdl90_send().
    s_udp_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);

    // Periodic deauth-nudge: force re-DHCP for associated-but-unresolved clients.
    const esp_timer_create_args_t nudge_args = {
        .callback = nudge_timer_cb,
        .name = "gdl90_nudge",
    };
    ESP_ERROR_CHECK(esp_timer_create(&nudge_args, &s_nudge_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(s_nudge_timer, NUDGE_PERIOD_US));

    // Prime the static GDL90 target cache from the saved CSV.
    if (net_set_static_dest(g_settings.gdl90_dest) < 0)
        ESP_LOGW(TAG, "ignoring invalid gdl90_dest setting: \"%s\"",
                 g_settings.gdl90_dest);

    ESP_LOGI(TAG, "SoftAP up: SSID=%s chan=%u auth=%s; GDL90 unicast -> leases:%d",
             g_settings.wifi_ssid, wc.ap.channel,
             g_settings.wifi_pass[0] ? "WPA2" : "open", GDL90_UDP_PORT);
}

void net_sta_status(net_sta_status_t *out)
{
    memset(out, 0, sizeof(*out));
    out->enabled = g_settings.sta_en;
    uint32_t ip = s_sta_ip, gw = s_sta_gw;
    out->connected = ip != 0;
    snprintf(out->ip, sizeof(out->ip), IPSTR, IP2STR((ip4_addr_t *)&ip));
    snprintf(out->gw, sizeof(out->gw), IPSTR, IP2STR((ip4_addr_t *)&gw));
    uint32_t dns = 0;
    if (out->connected && s_sta_netif) {
        esp_netif_dns_info_t di;
        if (esp_netif_get_dns_info(s_sta_netif, ESP_NETIF_DNS_MAIN, &di) == ESP_OK)
            dns = di.ip.u_addr.ip4.addr;
    }
    snprintf(out->dns, sizeof(out->dns), IPSTR, IP2STR((ip4_addr_t *)&dns));
}

int net_set_static_dest(const char *csv)
{
    uint32_t tmp[MAX_STATIC_DEST];
    int n = 0;

    // Validate the whole list into tmp before touching the live cache.
    if (csv && csv[0]) {
        char buf[64];
        strlcpy(buf, csv, sizeof(buf));
        char *save = NULL;
        for (char *tok = strtok_r(buf, ",", &save); tok;
             tok = strtok_r(NULL, ",", &save)) {
            struct in_addr a;
            if (n >= MAX_STATIC_DEST || !inet_aton(tok, &a) || a.s_addr == 0)
                return -1;
            tmp[n++] = a.s_addr;
        }
    }

    xSemaphoreTake(s_lease_mux, portMAX_DELAY);
    memcpy(s_static_dest, tmp, n * sizeof(tmp[0]));
    s_static_dest_n = n;
    xSemaphoreGive(s_lease_mux);
    return n;
}

int net_client_count(void)
{
    wifi_sta_list_t sta;
    if (esp_wifi_ap_get_sta_list(&sta) == ESP_OK) return sta.num;
    return 0;
}

// Build the set of GDL90 unicast destinations (network-byte-order IPs, deduped).
//
// Three sources, unioned: (1) our event cache, populated when STAIPASSIGNED
// fires; (1b) the user's static target list (g_settings.gdl90_dest — typically
// hosts on the STA-joined network; the unbound UDP socket routes per
// destination, so AP-subnet and STA-subnet targets both work);
// (2) the SoftAP DHCP server's live bindings for every currently-associated
// station, resolved via esp_netif_dhcps_get_clients_by_mac() (the ESP-IDF analog
// of Stratux getDHCPLeases()). Polling the server each send is more robust than
// the event cache alone: it survives missed events and lease renewals, and stays
// strictly unicast. (Caveat: after an ESP32 reboot the server's lease memory is
// also lost, so a client that keeps its cached IP without re-DHCPing won't be
// resolvable until it renews.)
static int collect_dest_ips(uint32_t *ips, int cap)
{
    int n = 0;
    #define ADD_IP(addr) do { \
        uint32_t _a = (addr); \
        if (_a) { int _dup = 0; \
            for (int _j = 0; _j < n; _j++) if (ips[_j] == _a) { _dup = 1; break; } \
            if (!_dup && n < cap) ips[n++] = _a; } \
    } while (0)

    if (s_lease_mux) {
        xSemaphoreTake(s_lease_mux, portMAX_DELAY);
        for (int i = 0; i < MAX_CLIENTS; i++)
            if (s_leases[i].used) ADD_IP(s_leases[i].ip);
        for (int i = 0; i < s_static_dest_n; i++)
            ADD_IP(s_static_dest[i]);
        xSemaphoreGive(s_lease_mux);
    }

    wifi_sta_list_t sta;
    if (s_ap_netif && esp_wifi_ap_get_sta_list(&sta) == ESP_OK && sta.num > 0) {
        int m = sta.num < MAX_CLIENTS ? sta.num : MAX_CLIENTS;
        esp_netif_pair_mac_ip_t pairs[MAX_CLIENTS];
        memset(pairs, 0, sizeof(pairs));
        for (int i = 0; i < m; i++)
            memcpy(pairs[i].mac, sta.sta[i].mac, 6);
        if (esp_netif_dhcps_get_clients_by_mac(s_ap_netif, m, pairs) == ESP_OK) {
            for (int i = 0; i < m; i++)
                ADD_IP(pairs[i].ip.addr);
        }
    }

    #undef ADD_IP
    return n;
}

int net_lease_count(void)
{
    uint32_t ips[MAX_DEST];
    return collect_dest_ips(ips, MAX_DEST);
}

void net_gdl90_send(const uint8_t *datagram, size_t len)
{
    if (s_udp_sock < 0 || !datagram || !len) return;

    uint32_t ips[MAX_DEST];
    int n = collect_dest_ips(ips, MAX_DEST);

    struct sockaddr_in dst;
    memset(&dst, 0, sizeof(dst));
    dst.sin_family = AF_INET;
    dst.sin_port   = htons(GDL90_UDP_PORT);
    for (int i = 0; i < n; i++) {
        dst.sin_addr.s_addr = ips[i];
        sendto(s_udp_sock, datagram, len, 0,
               (struct sockaddr *)&dst, sizeof(dst));
    }
}
