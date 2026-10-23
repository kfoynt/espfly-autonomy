#include <string.h>

#include "config.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>

#include "queuemonitor.h"
#include "wifi_esp32.h"
#include "stm32_legacy.h"
#define DEBUG_MODULE  "WIFI_UDP"
#include "debug_cf.h"

#define UDP_SERVER_PORT         2390
#define UDP_SERVER_BUFSIZE      128

static struct sockaddr_in6 source_addr; // Large enough for both IPv4 or IPv6

//#define WIFI_SSID      "Udp Server"
static char WIFI_SSID[32] = "ESP-DRONE";
static char WIFI_PWD[64] = "12345678" ;
static uint8_t WIFI_CH = 1;
#define MAX_STA_CONN (3)
#define WIFI_TX_POWER_QDBM 44   /* 11 dBm, in 0.25 dBm units */
#define WIFI_TX_POWER_LOW_QDBM 8   /* 2 dBm, the minimum: take-off, phone close by */
#define WIFI_QUIET_BEACON_TU 300

static bool quietOn;
static volatile bool txLowOn;

#ifndef MAC2STR
#define MAC2STR(a) (a)[0], (a)[1], (a)[2], (a)[3], (a)[4], (a)[5]
#define MACSTR "%02x:%02x:%02x:%02x:%02x:%02x"
#endif

static char rx_buffer[UDP_SERVER_BUFSIZE];
static char tx_buffer[UDP_SERVER_BUFSIZE];
const int addr_family = (int)AF_INET;
const int ip_protocol = IPPROTO_IP;
static struct sockaddr_in dest_addr;
static int sock;

static xQueueHandle udpDataRx;
static xQueueHandle udpDataTx;
static UDPPacket inPacket;
static UDPPacket outPacket;

static bool isInit = false;
static bool isUDPInit = false;
static bool isUDPConnected = false;

static esp_err_t udp_server_create(void *arg);

static uint8_t calculate_cksum(void *data, size_t len)
{
    unsigned char *c = data;
    int i;
    unsigned char cksum = 0;

    for (i = 0; i < len; i++) {
        cksum += *(c++);
    }

    return cksum;
}

/* Link diagnostics, read by the USB console. */
static volatile wifiLinkStats_t linkStats;

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    if (event_id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t *event = (wifi_event_ap_staconnected_t *) event_data;
        linkStats.staJoins++;
        linkStats.lastJoinMs = xTaskGetTickCount() * portTICK_PERIOD_MS;
        linkStats.appFirstMs = 0;   /* armed: set by the next valid app packet */
        DEBUG_PRINT_LOCAL("station" MACSTR "join, AID=%d", MAC2STR(event->mac), event->aid);

    } else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t *event = (wifi_event_ap_stadisconnected_t *) event_data;
        linkStats.staLeaves++;
        linkStats.lastLeaveMs = xTaskGetTickCount() * portTICK_PERIOD_MS;
        DEBUG_PRINT_LOCAL("station" MACSTR "leave, AID=%d", MAC2STR(event->mac), event->aid);
    } 
}

void wifiGetLinkStats(wifiLinkStats_t *out)
{
    *out = linkStats;
    int8_t p = 0;
    out->txPowerQdbm = (isInit && esp_wifi_get_max_tx_power(&p) == ESP_OK) ? p : 0;
}

unsigned wifiApBandwidthMhz(void)
{
    wifi_bandwidth_t bw = WIFI_BW_HT20;
    if (!isInit || esp_wifi_get_bandwidth(WIFI_IF_AP, &bw) != ESP_OK) {
        return 0;
    }
    return bw == WIFI_BW_HT20 ? 20 : 40;
}

void wifiGetCounters(uint32_t *txPackets, uint32_t *rxPackets, uint32_t *lastJoinMs, uint32_t *appFirstMs)
{
    *txPackets = linkStats.txPackets;
    *rxPackets = linkStats.rxPackets;
    *lastJoinMs = linkStats.lastJoinMs;
    *appFirstMs = linkStats.appFirstMs;
}

bool wifiTest(void)
{
    return isInit;
};

bool wifiGetDataBlocking(UDPPacket *in)
{
    /* command step - receive  02  from udp rx queue */
    while (xQueueReceive(udpDataRx, in, portMAX_DELAY) != pdTRUE) {
        vTaskDelay(1);
    }; // Don't return until we get some data on the UDP

    return true;
};

bool wifiSendData(uint32_t size, uint8_t *data)
{
    static UDPPacket outStage;
    outStage.size = size;
    memcpy(outStage.data, data, size);
    // Dont' block when sending
    return (xQueueSend(udpDataTx, &outStage, M2T(100)) == pdTRUE);
};

static esp_err_t udp_server_create(void *arg)
{ 
    if (isUDPInit){
        return ESP_OK;
    }
    
    struct sockaddr_in *pdest_addr = &dest_addr;
    pdest_addr->sin_addr.s_addr = htonl(INADDR_ANY);
    pdest_addr->sin_family = AF_INET;
    pdest_addr->sin_port = htons(UDP_SERVER_PORT);

    sock = socket(addr_family, SOCK_DGRAM, ip_protocol);
    if (sock < 0) {
        DEBUG_PRINT_LOCAL("Unable to create socket: errno %d", errno);
        return ESP_FAIL;
    }
    DEBUG_PRINT_LOCAL("Socket created");

    int err = bind(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
    if (err < 0) {
        DEBUG_PRINT_LOCAL("Socket unable to bind: errno %d", errno);
    }
    DEBUG_PRINT_LOCAL("Socket bound, port %d", UDP_SERVER_PORT);

    isUDPInit = true;
    return ESP_OK;
}

static void udp_server_rx_task(void *pvParameters)
{
    uint8_t cksum = 0;
    socklen_t socklen = sizeof(source_addr);
    
    while (true) {
        if(isUDPInit == false) {
            vTaskDelay(20);
            continue;
        }
        int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer) - 1, 0, (struct sockaddr *)&source_addr, &socklen);
        /* command step - receive  01 from Wi-Fi UDP */
        if (len < 0) {
            /* Previously 'break': the task function then returned, which
             * aborts (reboots) the drone. Keep receiving instead. */
            linkStats.rxErrors++;
            linkStats.lastRxErrno = errno;
            DEBUG_PRINT_LOCAL("recvfrom failed: errno %d", errno);
            vTaskDelay(M2T(10));
            continue;
        } else if (len < 2) {
            // A CRTP datagram needs at least a header plus checksum. In
            // particular, an empty UDP packet must never index data[-1].
            continue;
        } else if(len > WIFI_RX_TX_PACKET_SIZE - 4) {
            DEBUG_PRINT_LOCAL("Received data length = %d > 64", len);
        } else {
            //copy part of the UDP packet
            rx_buffer[len] = 0;// Null-terminate whatever we received and treat like a string...
            memcpy(inPacket.data, rx_buffer, len);
            cksum = inPacket.data[len - 1];
            //remove cksum, do not belong to CRTP
            inPacket.size = len - 1;
            //check packet
            if (cksum == calculate_cksum(inPacket.data, len - 1) && inPacket.size < 64){
                linkStats.rxPackets++;
                if (linkStats.appFirstMs == 0) {
                    uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
                    linkStats.appFirstMs = now ? now : 1;
                }
                if (xQueueSend(udpDataRx, &inPacket, M2T(2)) != pdTRUE) {
                    linkStats.queueDrops++;
                }
                if(!isUDPConnected) isUDPConnected = true;
            }else{
                linkStats.cksumErrors++;
                DEBUG_PRINT_LOCAL("udp packet cksum unmatched");
            }

#ifdef DEBUG_UDP
            DEBUG_PRINT_LOCAL("1.Received data size = %d  %02X \n cksum = %02X", len, inPacket.data[0], cksum);
            for (size_t i = 0; i < len; i++) {
                DEBUG_PRINT_LOCAL(" data[%d] = %02X ", i, inPacket.data[i]);
            }
#endif
        }
    }
}

static void udp_server_tx_task(void *pvParameters)
{
 
    while (TRUE) {
        if(isUDPInit == false) {
            vTaskDelay(20);
            continue;
        }
        if ((xQueueReceive(udpDataTx, &outPacket, 5) == pdTRUE) && isUDPConnected) {           
            memcpy(tx_buffer, outPacket.data, outPacket.size);       
            tx_buffer[outPacket.size] =  calculate_cksum(tx_buffer, outPacket.size);
            tx_buffer[outPacket.size + 1] = 0;

            int err = sendto(sock, tx_buffer, outPacket.size + 1, 0, (struct sockaddr *)&source_addr, sizeof(source_addr));
            if (err < 0) {
                linkStats.txErrors++;
                DEBUG_PRINT_LOCAL("Error occurred during sending: errno %d", errno);
                continue;
            }
            linkStats.txPackets++;
#ifdef DEBUG_UDP
            DEBUG_PRINT_LOCAL("Send data to");
            for (size_t i = 0; i < outPacket.size + 1; i++) {
                DEBUG_PRINT_LOCAL(" data_send[%d] = %02X ", i, tx_buffer[i]);
            }
#endif
        }    
    }
}


void wifiSetQuiet(bool on)
{
    quietOn = on;
}

bool wifiQuiet(void)
{
    return quietOn;
}

esp_err_t wifiSetTxLow(bool low)
{
    if (!isInit) {
        return ESP_ERR_INVALID_STATE;
    }
    esp_err_t err = esp_wifi_set_max_tx_power(low ? WIFI_TX_POWER_LOW_QDBM : WIFI_TX_POWER_QDBM);
    if (err == ESP_OK) {
        txLowOn = low;
    }
    return err;
}

bool wifiTxLow(void)
{
    return txLowOn;
}

/* HT20 for this boot only, never saved. With the default storage
 * (WIFI_STORAGE_FLASH; CONFIG_ESP32_WIFI_NVS_ENABLED=y) the driver saves the
 * bandwidth to its own NVS namespace (nvs.net80211: ap.phybw, and
 * ap.sndchan = none), and esp_wifi_init() loads it on every later boot,
 * older builds included: flashing back an earlier build would then keep
 * HT20, and undoing that would need an NVS erase, which also loses the
 * calibration. Under WIFI_STORAGE_RAM the driver changes only its RAM copy
 * (IDF 5.0.7 libnet80211: ieee80211_set_phy_bw passes "store = storage is
 * FLASH" to wifi_nvs_set, and wifi_nvs_commit skips under RAM), so this
 * build runs HT20 and every other build starts at HT40 as before. If RAM
 * storage cannot be selected the bandwidth is left alone (HT40 stays).
 * FLASH is restored afterwards, as for every build before this one. */
static esp_err_t apSetHt20ThisBoot(void)
{
    esp_err_t err = esp_wifi_set_storage(WIFI_STORAGE_RAM);
    if (err != ESP_OK) {
        return err;
    }
    err = esp_wifi_set_bandwidth(WIFI_IF_AP, WIFI_BW_HT20);
    if (esp_wifi_set_storage(WIFI_STORAGE_FLASH) != ESP_OK) {
        DEBUG_PRINT_LOCAL("wifi: could not restore the flash storage of the Wi-Fi settings");
    }
    return err;
}

void wifiInit(void)
{
    if (isInit) {
        return;
    }

    esp_netif_t *ap_netif = NULL;
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ap_netif = esp_netif_create_default_wifi_ap();
    uint8_t mac[6];

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                    ESP_EVENT_ANY_ID,
                    &wifi_event_handler,
                    NULL,
                    NULL));

    ESP_ERROR_CHECK(esp_wifi_get_mac(ESP_IF_WIFI_AP, mac));
    sprintf(WIFI_SSID, "ESP-DRONE_%02X%02X%02X%02X%02X%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    wifi_config_t wifi_config = {
        .ap = {
            .channel = WIFI_CH,
            .max_connection = MAX_STA_CONN,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK,
        },
    };

    memcpy(wifi_config.ap.ssid, WIFI_SSID, strlen(WIFI_SSID) + 1) ;
    wifi_config.ap.ssid_len = strlen(WIFI_SSID);
    memcpy(wifi_config.ap.password, WIFI_PWD, strlen(WIFI_PWD) + 1) ;

    if (strlen(WIFI_PWD) == 0) {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }
    if (quietOn) {
        wifi_config.ap.beacon_interval = WIFI_QUIET_BEACON_TU;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_AP, &wifi_config));
    /* 20 MHz (HT20) instead of IDF 5.0.7's soft-AP default HT40 (user
     * decision 2026-10-13, evening): about 3 mA less receive current when a
     * station would use 40 MHz (datasheet 91 -> 88 mA), and a link that
     * shares a busy 2.4 GHz band better. 72 Mbps at most, far above the
     * link's needs. Set once the AP interface exists, before the start; the
     * channel (WIFI_CH, now without the secondary above it), TX power, the
     * take-off low TX power and the quiet option are unchanged. A failure
     * keeps HT40 (logged) and never stops the boot. Not saved to the Wi-Fi
     * NVS (apSetHt20ThisBoot), so an older build runs HT40 again. */
    const esp_err_t bwErr = apSetHt20ThisBoot();
    if (quietOn && esp_wifi_config_11b_rate(WIFI_IF_AP, true) != ESP_OK) {
        DEBUG_PRINT_LOCAL("wifi: could not disable 11b rates");
        quietOn = false;   /* report what is in effect (the beacon interval alone stays) */
    }
    ESP_ERROR_CHECK(esp_wifi_start());
    {
        wifi_bandwidth_t bw = WIFI_BW_HT40;
        esp_err_t err = esp_wifi_get_bandwidth(WIFI_IF_AP, &bw);
        if (err == ESP_OK && bw != WIFI_BW_HT20) {   /* not kept through the start: once more */
            err = apSetHt20ThisBoot();
            if (err == ESP_OK) {
                err = esp_wifi_get_bandwidth(WIFI_IF_AP, &bw);
            }
        }
        uint8_t primary = 0;
        wifi_second_chan_t second = WIFI_SECOND_CHAN_NONE;
        (void)esp_wifi_get_channel(&primary, &second);
        DEBUG_PRINT_LOCAL("wifi: AP bandwidth %s (set: %s, read: %s, not saved), channel %u, secondary %d",
                          err != ESP_OK ? "?" : (bw == WIFI_BW_HT20 ? "HT20" : "HT40"),
                          esp_err_to_name(bwErr), esp_err_to_name(err), (unsigned)primary, (int)second);
    }

    /* 11 dBm instead of the 20 dBm default: the phone is a few metres away,
     * and every transmit burst at 20 dBm draws ~300 mA from the same 3.3 V
     * regulator as the chip, which has no headroom when the battery sags to
     * ~3.0 V at lift-off (the drone browned out there on 2026-10-11). */
    if (esp_wifi_set_max_tx_power(WIFI_TX_POWER_QDBM) != ESP_OK) {
        DEBUG_PRINT_LOCAL("wifi: could not lower TX power");
    }

    esp_netif_ip_info_t ip_info = {
        .ip.addr = ipaddr_addr("192.168.43.42"),
        .netmask.addr = ipaddr_addr("255.255.255.0"),
        .gw.addr      = ipaddr_addr("192.168.43.42"),
    };
    ESP_ERROR_CHECK(esp_netif_dhcps_stop(ap_netif));
    ESP_ERROR_CHECK(esp_netif_set_ip_info(ap_netif, &ip_info));
    ESP_ERROR_CHECK(esp_netif_dhcps_start(ap_netif));

    DEBUG_PRINT_LOCAL("wifi_init_softap complete.SSID:%s password:%s", WIFI_SSID, WIFI_PWD);

    // This should probably be reduced to a CRTP packet size
    udpDataRx = xQueueCreate(5, sizeof(UDPPacket)); /* Buffer packets (max 64 bytes) */
    DEBUG_QUEUE_MONITOR_REGISTER(udpDataRx);
    udpDataTx = xQueueCreate(5, sizeof(UDPPacket)); /* Buffer packets (max 64 bytes) */
    DEBUG_QUEUE_MONITOR_REGISTER(udpDataTx);
    if (udp_server_create(NULL) == ESP_FAIL) {
        DEBUG_PRINT_LOCAL("UDP server create socket failed!!!");
    } else {
        DEBUG_PRINT_LOCAL("UDP server create socket succeed!!!");
    } 
    xTaskCreate(udp_server_tx_task, UDP_TX_TASK_NAME, UDP_TX_TASK_STACKSIZE, NULL, UDP_TX_TASK_PRI, NULL);
    xTaskCreate(udp_server_rx_task, UDP_RX_TASK_NAME, UDP_RX_TASK_STACKSIZE, NULL, UDP_RX_TASK_PRI, NULL);
    isInit = true;
}
