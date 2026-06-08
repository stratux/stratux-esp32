#include "net.h"
#include <string.h>
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "lwip/sockets.h"
#include "settings.h"

static const char *TAG = "net";

#define GDL90_UDP_PORT   4000
#define AP_BROADCAST_IP  "192.168.10.255"   // M0 broadcast target

static int               s_udp_sock = -1;
static struct sockaddr_in s_bcast;

void net_wifi_softap_start(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_t *ap = esp_netif_create_default_wifi_ap();
    (void)ap;

    // TODO(M0): set AP IP to 192.168.10.1 (esp_netif_set_ip_info with a stopped
    // DHCP server, then restart it) so the gateway matches Stratux defaults.

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

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

    // UDP sender for GDL90. M0: broadcast to the AP subnet. M2: per-station
    // unicast (track IP_EVENT_AP_STAIPASSIGNED).
    s_udp_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    int yes = 1;
    setsockopt(s_udp_sock, SOL_SOCKET, SO_BROADCAST, &yes, sizeof(yes));
    memset(&s_bcast, 0, sizeof(s_bcast));
    s_bcast.sin_family = AF_INET;
    s_bcast.sin_port   = htons(GDL90_UDP_PORT);
    s_bcast.sin_addr.s_addr = inet_addr(AP_BROADCAST_IP);

    ESP_LOGI(TAG, "SoftAP up: SSID=%s chan=%u auth=%s; GDL90 -> %s:%d",
             g_settings.wifi_ssid, wc.ap.channel,
             g_settings.wifi_pass[0] ? "WPA2" : "open",
             AP_BROADCAST_IP, GDL90_UDP_PORT);
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
    // TODO(M2): replace broadcast with a unicast loop over associated stations.
    sendto(s_udp_sock, datagram, len, 0,
           (struct sockaddr *)&s_bcast, sizeof(s_bcast));
}
