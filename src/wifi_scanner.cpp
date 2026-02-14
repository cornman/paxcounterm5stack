// ─────────────────────────────────────────────────────────────────────────────
// PAX Counter M5Stack - WiFi Probe Request Scanner
// ─────────────────────────────────────────────────────────────────────────────
#include "wifi_scanner.h"

#if ENABLE_WIFI_SCAN

#include "config.h"
#include "types.h"
#include "pax_store.h"

#include <WiFi.h>
#include <esp_wifi.h>
#include <freertos/semphr.h>

// ── Captured MAC entry ───────────────────────────────────────────────────────
struct WifiCapture {
    uint64_t mac;
    int      rssi;
};

static constexpr size_t CAPTURE_BUF_SIZE = 128;
static WifiCapture      g_captures[CAPTURE_BUF_SIZE];
static volatile size_t  g_captureCount = 0;
static SemaphoreHandle_t g_mutex       = nullptr;
static uint8_t           g_channel     = 1;

// ── 802.11 management frame header (minimal) ────────────────────────────────
typedef struct {
    uint8_t  frame_ctrl[2];
    uint8_t  duration[2];
    uint8_t  addr1[6];     // Destination
    uint8_t  addr2[6];     // Source — this is the device MAC
    uint8_t  addr3[6];     // BSSID
    uint8_t  seq_ctrl[2];
} __attribute__((packed)) wifi_mgmt_hdr_t;

// ── Promiscuous callback (runs in WiFi task, NOT main loop) ──────────────────
static void IRAM_ATTR snifferCb(void* buf, wifi_promiscuous_pkt_type_t type) {
    if (type != WIFI_PKT_MGMT) return;

    const wifi_promiscuous_pkt_t* pkt = (wifi_promiscuous_pkt_t*)buf;
    const wifi_mgmt_hdr_t* hdr       = (wifi_mgmt_hdr_t*)pkt->payload;

    // Probe Request: frame type=Management(00), subtype=0100 → byte 0 = 0x40
    if ((hdr->frame_ctrl[0] & 0xFC) != 0x40) return;

    // Build uint64_t from source MAC (addr2)
    uint64_t mac = 0;
    for (int i = 0; i < 6; i++)
        mac = (mac << 8) | hdr->addr2[i];

    // Skip null MAC
    if (mac == 0) return;

    int rssi = pkt->rx_ctrl.rssi;

    if (xSemaphoreTakeFromISR(g_mutex, nullptr) == pdTRUE) {
        if (g_captureCount < CAPTURE_BUF_SIZE) {
            g_captures[g_captureCount++] = {mac, rssi};
        }
        xSemaphoreGiveFromISR(g_mutex, nullptr);
    }
}

// ── Public API ───────────────────────────────────────────────────────────────

void initWifiScanner() {
    g_mutex = xSemaphoreCreateMutex();

    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    delay(100);

    esp_wifi_set_promiscuous(true);
    esp_wifi_set_promiscuous_rx_cb(&snifferCb);
    esp_wifi_set_channel(g_channel, WIFI_SECOND_CHAN_NONE);

    Serial.println("[WiFi] Promiscuous mode active, sniffing probe requests");
}

void hopWifiChannel() {
    g_channel++;
    if (g_channel > WIFI_NUM_CHANNELS) g_channel = 1;
    esp_wifi_set_channel(g_channel, WIFI_SECOND_CHAN_NONE);
}

void drainWifiCaptures() {
    if (!g_mutex) return;
    if (xSemaphoreTake(g_mutex, pdMS_TO_TICKS(5)) != pdTRUE) return;

    // Copy out under lock, then release quickly
    size_t n = g_captureCount;
    WifiCapture local[CAPTURE_BUF_SIZE];
    if (n > 0) {
        memcpy(local, g_captures, n * sizeof(WifiCapture));
        g_captureCount = 0;
    }
    xSemaphoreGive(g_mutex);

    // Ingest into pax_store outside the lock
    for (size_t i = 0; i < n; i++) {
        std::string macStr = macToString(local[i].mac);
        paxStoreIngest(local[i].mac, macStr, CLS_WIFI_PROBE,
                       local[i].rssi, ScanSource::WIFI, true);
    }
}

#endif // ENABLE_WIFI_SCAN
