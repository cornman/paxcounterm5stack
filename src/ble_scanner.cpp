// ─────────────────────────────────────────────────────────────────────────────
// PAX Counter M5Stack - BLE Scanner
// ─────────────────────────────────────────────────────────────────────────────
#include "ble_scanner.h"
#include "config.h"
#include "types.h"
#include "device_classifier.h"
#include "pax_store.h"

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <set>

#ifndef ESP_BD_ADDR_LEN
#define ESP_BD_ADDR_LEN 6
#endif

static BLEScan* pScan = nullptr;

static uint64_t addressToKey(BLEAddress& addr) {
    const esp_bd_addr_t* raw = addr.getNative();
    if (!raw) return 0;
    uint64_t v = 0;
    for (int i = 0; i < ESP_BD_ADDR_LEN; i++)
        v = (v << 8) | (*raw)[i];
    return v;
}

bool initBle() {
    BLEDevice::init("");
    pScan = BLEDevice::getScan();
    if (!pScan) return false;
    pScan->setActiveScan(true);
    pScan->setInterval(BLE_SCAN_INTERVAL);
    pScan->setWindow(BLE_SCAN_WINDOW);
    return true;
}

void runBleScan() {
    if (!pScan) return;

    BLEScanResults* results = pScan->start(BLE_SCAN_TIME_S, false);
    if (!results || results->getCount() == 0) {
        if (results) pScan->clearResults();
        return;
    }

    std::set<uint64_t> seen;
    for (int i = 0; i < results->getCount(); i++) {
        BLEAdvertisedDevice dev = results->getDevice(i);
        BLEAddress addr = dev.getAddress();
        uint64_t key = addressToKey(addr);
        if (key == 0) continue;

        std::string label = classifyDevice(dev);
        int rssi = dev.getRSSI();
        std::string macStr = addr.toString().c_str();

        bool firstTimestamp = (seen.find(key) == seen.end());
        if (firstTimestamp) seen.insert(key);

        paxStoreIngest(key, macStr, label, rssi, ScanSource::BLE, firstTimestamp);
    }
    pScan->clearResults();
}
