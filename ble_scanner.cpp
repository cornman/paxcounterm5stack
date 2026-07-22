// ─────────────────────────────────────────────────────────────────────────────
// BLE Scanner
// ─────────────────────────────────────────────────────────────────────────────
#include "ble_scanner.h"
#include "config.h"
#include "types.h"
#include "classification.h"
#include "device_classifier.h"
#include "pax_store.h"
#include "c_assert.h"

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>

#ifndef ESP_BD_ADDR_LEN
#define ESP_BD_ADDR_LEN 6
#endif

static BLEScan* g_scan = nullptr;

// Fixed dedupe set for one scan cycle (avoids double-counting a device that the
// stack reports twice). Bounded by BLE_MAX_RESULTS — no std::set (Rule 3).
static uint64_t g_seen[BLE_MAX_RESULTS];
static size_t   g_seen_n = 0;

static uint64_t addressToKey(BLEAddress& addr) {
    const esp_bd_addr_t* raw = addr.getNative();
    if (!c_assert(raw != nullptr)) {
        return 0;
    }
    uint64_t v = 0;
    for (int i = 0; i < ESP_BD_ADDR_LEN; i++) {      // bound: 6 (Rule 2)
        v = (v << 8) | (*raw)[i];
    }
    return v;
}

// Returns true if `key` was newly added this cycle (i.e. record a timestamp).
static bool seenAdd(uint64_t key) {
    for (size_t i = 0; i < g_seen_n; ++i) {          // bound: BLE_MAX_RESULTS
        if (g_seen[i] == key) {
            return false;
        }
    }
    if (g_seen_n < BLE_MAX_RESULTS) {
        g_seen[g_seen_n++] = key;
        return true;
    }
    return false;                                    // set full: treat as seen
}

bool initBle() {
    BLEDevice::init("");
    g_scan = BLEDevice::getScan();
    if (!c_assert(g_scan != nullptr)) {
        return false;
    }
    g_scan->setActiveScan(true);
    g_scan->setInterval(BLE_SCAN_INTERVAL);
    g_scan->setWindow(BLE_SCAN_WINDOW);
    return true;
}

void runBleScan(uint32_t now_ms) {
    if (!c_assert(g_scan != nullptr)) {
        return;
    }
    BLEScanResults* results = g_scan->start(BLE_SCAN_TIME_S, false);
    if (results == nullptr) {
        Serial.println("[BLE] scan returned null");
        return;
    }
    int count = results->getCount();
    if (!c_assert(count >= 0)) {
        g_scan->clearResults();
        return;
    }
    int limit = count;
    if (limit > (int)BLE_MAX_RESULTS) {              // no silent truncation
        Serial.printf("[BLE] %d results, capping at %u\n",
                      count, (unsigned)BLE_MAX_RESULTS);
        limit = (int)BLE_MAX_RESULTS;
    }
    g_seen_n = 0;
    for (int i = 0; i < limit; ++i) {                // bound: BLE_MAX_RESULTS
        BLEAdvertisedDevice dev = results->getDevice(i);
        BLEAddress addr = dev.getAddress();
        uint64_t key = addressToKey(addr);
        if (key == 0) {
            continue;
        }
        char mac[MAC_STR_CAP];
        if (!macToStr(mac, sizeof(mac), key)) {
            continue;
        }
        uint8_t cls  = classifyDevice(dev);
        bool    first = seenAdd(key);
        // Drops (table full) are tracked by psDropped(); nothing to do here.
        (void)psIngest(key, mac, cls, dev.getRSSI(), SRC_BLE, first, now_ms);
    }
    g_scan->clearResults();
}
