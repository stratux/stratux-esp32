#include "net.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "lwip/sockets.h"
#include "settings.h"

static const char *TAG = "net";

#define GDL90_UDP_PORT   4000
#define MAX_CLIENTS      8        // SoftAP max_connection is 4; headroom for churn

static int               s_udp_sock = -1;
static SemaphoreHandle_t s_lease_mux;

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

static void net_event_handler(void *arg, esp_event_base_t base,
                              int32_t id, void *data)
{
    if (base == IP_EVENT && id == IP_EVENT_AP_STAIPASSIGNED) {
        ip_event_ap_staipassigned_t *e = (ip_event_ap_staipassigned_t *)data;
        lease_upsert(e->mac, e->ip.addr);
        ESP_LOGI(TAG, "EFB client lease " IPSTR, IP2STR(&e->ip));
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t *e = (wifi_event_ap_stadisconnected_t *)data;
        lease_remove(e->mac);
        ESP_LOGI(TAG, "EFB client disconnected; lease dropped");
    }
}

void net_wifi_softap_start(void)
{
    s_lease_mux = xSemaphoreCreateMutex();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_t *ap = esp_netif_create_default_wifi_ap();

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
        WIFI_EVENT, WIFI_EVENT_AP_STADISCONNECTED, net_event_handler, NULL, NULL));

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

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wc));
    ESP_ERROR_CHECK(esp_wifi_start());

    // GDL90 UDP sender. Datagrams are unicast per lease in net_gdl90_send().
    s_udp_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);

    ESP_LOGI(TAG, "SoftAP up: SSID=%s chan=%u auth=%s; GDL90 unicast -> leases:%d",
             g_settings.wifi_ssid, wc.ap.channel,
             g_settings.wifi_pass[0] ? "WPA2" : "open", GDL90_UDP_PORT);
}

int net_client_count(void)
{
    wifi_sta_list_t sta;
    if (esp_wifi_ap_get_sta_list(&sta) == ESP_OK) return sta.num;
    return 0;
}

void net_gdl90_send(const uint8_t *datagram, size_t len)
{
    if (s_udp_sock < 0 || !datagram || !len) return;

    // Snapshot lease IPs under the lock, then send outside it (a UDP sendto can
    // block briefly in lwIP; don't hold the lease mutex across it).
    uint32_t ips[MAX_CLIENTS];
    int n = 0;
    xSemaphoreTake(s_lease_mux, portMAX_DELAY);
    for (int i = 0; i < MAX_CLIENTS; i++) {
        if (s_leases[i].used) ips[n++] = s_leases[i].ip;
    }
    xSemaphoreGive(s_lease_mux);

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
