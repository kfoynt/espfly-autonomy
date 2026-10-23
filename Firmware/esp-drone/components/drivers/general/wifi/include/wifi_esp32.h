#ifndef WIFI_ESP32_H_
#define WIFI_ESP32_H_
#include <stdbool.h>
#include "esp_err.h"
#include <stdint.h>

#define WIFI_RX_TX_PACKET_SIZE   (64)

/* Structure used for in/out data via USB */
typedef struct
{
  uint8_t size;
  uint8_t data[WIFI_RX_TX_PACKET_SIZE];
} UDPPacket;

/**
 * Initialize the wifi.
 *
 * @note Initialize CRTP link only if USE_CRTP_WIFI is defined
 */
void wifiInit(void);

/* Call before wifiInit: no 11b rates (beacons, ACKs and management frames at
 * 6 Mbps OFDM, ~5x shorter transmit bursts than 1 Mbps) and a 300 TU beacon
 * interval (a third of the beacons). */
void wifiSetQuiet(bool on);
bool wifiQuiet(void);

/* Low TX power (2 dBm) for take-off, when the phone is close by, or back to
 * the normal 11 dBm. */
esp_err_t wifiSetTxLow(bool low);
bool wifiTxLow(void);

/**
 * Test the WIFI status.
 *
 * @return true if the WIFI is initialized
 */
bool wifiTest(void);

typedef struct {
    uint32_t staJoins;      /* phone/station joined the access point */
    uint32_t staLeaves;     /* station left (or was dropped) */
    uint32_t lastLeaveMs;   /* uptime of the last leave */
    uint32_t rxPackets;     /* valid UDP datagrams */
    uint32_t cksumErrors;
    uint32_t queueDrops;    /* RX queue full */
    uint32_t rxErrors;      /* recvfrom() failures */
    int32_t lastRxErrno;
    int8_t txPowerQdbm;     /* maximum TX power in effect, 0.25 dBm units */
    uint32_t lastJoinMs;    /* uptime of the last station join, 0 = none yet */
    uint32_t appFirstMs;    /* uptime of the first valid app packet after that join, 0 = none yet */
    uint32_t txPackets;     /* UDP datagrams sent to the app */
    uint32_t txErrors;      /* sendto() failures */
} wifiLinkStats_t;

void wifiGetLinkStats(wifiLinkStats_t *out);
/* The access point's bandwidth in effect: 20 (HT20, set at boot since
 * 2026-10-13, for that boot only: never saved to the Wi-Fi NVS, so an
 * older build runs HT40 again), 40 (HT40, the IDF default), 0 before init
 * or on an error. */
unsigned wifiApBandwidthMhz(void);
/* The counters above without any Wi-Fi driver call (safe at 100 Hz). */
void wifiGetCounters(uint32_t *txPackets, uint32_t *rxPackets, uint32_t *lastJoinMs, uint32_t *appFirstMs);

/**
 * Get CRTP link data structure
 *
 * @return Address of the crtp link operations structure.
 */
//struct crtpLinkOperations * wifiGetLink();

/**
 * Get data from rx queue with timeout.
 * @param[out] c  Byte of data
 *
 * @return true if byte received, false if timout reached.
 */
bool wifiGetDataBlocking(UDPPacket *in);

/**
 * Sends raw data using a lock. Should be used from
 * exception functions and for debugging when a lot of data
 * should be transfered.
 * @param[in] size  Number of bytes to send
 * @param[in] data  Pointer to data
 *
 * @note If WIFI Crtp link is activated this function does nothing
 */
bool wifiSendData(uint32_t size, uint8_t* data);

#endif