// ─────────────────────────────────────────────────────────────────────────────
// WiFi Probe Request Scanner
// ─────────────────────────────────────────────────────────────────────────────
#include "wifi_scanner.h"

#if ENABLE_WIFI_SCAN

#include "config.h"
#include "types.h"
#include "classification.h"
#include "pax_store.h"
#include "c_assert.h"

#include <Arduino.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <freertos/semphr.h>

struct WifiCapture {
    uint64_t mac;
    int      rssi;
};

static constexpr size_t CAPTURE_BUF_SIZE = 128;
static WifiCapture       g_captures[CAPTURE_BUF_SIZE];
static volatile size_t   g_captureCount = 0;
static SemaphoreHandle_t g_mutex        = nullptr;
static uint8_t           g_channel      = 1;

// Drain scratch (main-loop context only) — static, not on the stack (Rule 3).
static WifiCapture g_drain[CAPTURE_BUF_SIZE];

// Minimal 802.11 management-frame header; addr2 is the transmitter (device) MAC.
typedef struct {
    uint8_t frame_ctrl[2];
    uint8_t duration[2];
    uint8_t addr1[6];
    uint8_t addr2[6];
    uint8_t addr3[6];
    uint8_t seq_ctrl[2];
} __attribute__((packed)) wifi_mgmt_hdr_t;

// ── Promiscuous RX callback ──────────────────────────────────────────────────
// Runs in the WiFi task. It must stay ISR-simple: NO c_assert here (the assert
// macro logs via Serial, which is not safe in this context). It only copies a
// MAC + RSSI into a mutex-guarded fixed ring; all validation happens in the
// main-loop drain below.
static void snifferCb(void* buf, wifi_promiscuous_pkt_type_t type) {
    if (type != WIFI_PKT_MGMT || buf == nullptr) {
        return;
    }
    const wifi_promiscuous_pkt_t* pkt = (const wifi_promiscuous_pkt_t*)buf;
    const wifi_mgmt_hdr_t* hdr = (const wifi_mgmt_hdr_t*)pkt->payload;

    if ((hdr->frame_ctrl[0] & 0xFC) != 0x40) {       // probe request subtype
        return;
    }
    uint64_t mac = 0;
    for (int i = 0; i < 6; i++) {                    // bound: 6 (Rule 2)
        mac = (mac << 8) | hdr->addr2[i];
    }
    if (mac == 0) {
        return;
    }
    if (xSemaphoreTakeFromISR(g_mutex, nullptr) == pdTRUE) {
        if (g_captureCount < CAPTURE_BUF_SIZE) {
            g_captures[g_captureCount].mac  = mac;
            g_captures[g_captureCount].rssi = pkt->rx_ctrl.rssi;
            g_captureCount++;
        }
        xSemaphoreGiveFromISR(g_mutex, nullptr);
    }
}

// ── Public API ───────────────────────────────────────────────────────────────
bool initWifiScanner() {
    g_mutex = xSemaphoreCreateMutex();
    if (!c_assert(g_mutex != nullptr)) {
        return false;
    }
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    delay(100);

    if (!c_assert(esp_wifi_set_promiscuous(true) == ESP_OK)) {
        return false;
    }
    // Rule 9 WAIVER: esp_wifi_set_promiscuous_rx_cb takes a function pointer.
    // This is mandated by ESP-IDF and cannot be avoided. Mitigation: exactly one
    // static callback, never reassigned, doing minimal ISR-simple work — so the
    // call graph remains statically knowable in practice. See POWER_OF_TEN.md.
    if (!c_assert(esp_wifi_set_promiscuous_rx_cb(&snifferCb) == ESP_OK)) {
        return false;
    }
    if (!c_assert(esp_wifi_set_channel(g_channel, WIFI_SECOND_CHAN_NONE) == ESP_OK)) {
        return false;
    }
    Serial.println("[WiFi] promiscuous sniffing active");
    return true;
}

void hopWifiChannel() {
    g_channel++;
    if (g_channel > WIFI_NUM_CHANNELS) {
        g_channel = 1;
    }
    esp_err_t e = esp_wifi_set_channel(g_channel, WIFI_SECOND_CHAN_NONE);
    (void)c_assert(e == ESP_OK);
}

void drainWifiCaptures(uint32_t now_ms) {
    if (g_mutex == nullptr) {
        return;
    }
    if (xSemaphoreTake(g_mutex, pdMS_TO_TICKS(5)) != pdTRUE) {
        return;
    }
    size_t n = g_captureCount;
    if (n > CAPTURE_BUF_SIZE) {
        n = CAPTURE_BUF_SIZE;                        // defensive clamp
    }
    for (size_t i = 0; i < n; ++i) {                 // bound: CAPTURE_BUF_SIZE
        g_drain[i] = g_captures[i];
    }
    g_captureCount = 0;
    xSemaphoreGive(g_mutex);

    for (size_t i = 0; i < n; ++i) {                 // bound: CAPTURE_BUF_SIZE
        char mac[MAC_STR_CAP];
        if (!macToStr(mac, sizeof(mac), g_drain[i].mac)) {
            continue;
        }
        (void)psIngest(g_drain[i].mac, mac, CLS_WIFI, g_drain[i].rssi,
                       SRC_WIFI, true, now_ms);
    }
}

#endif  // ENABLE_WIFI_SCAN
