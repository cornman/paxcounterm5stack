// ─────────────────────────────────────────────────────────────────────────────
// PAX Counter M5Stack
// ─────────────────────────────────────────────────────────────────────────────
// Counts nearby people by detecting BLE advertisements AND WiFi probe
// requests, then displays the count, a 60-minute history graph, and
// detailed device lists on the M5Stack Core2 screen.
//
// Arduino IDE setup:
//   Board:    M5Stack Core2  (via ESP32 board package)
//   Libraries: M5Unified, ArduinoJson (install from Library Manager)
//
// Architecture (all files live in this sketch folder):
//   paxcounterm5stack.ino  – setup / loop orchestration (this file)
//   ble_scanner            – ESP32 BLE scanning
//   wifi_scanner           – ESP32 WiFi promiscuous-mode probe-request capture
//   device_classifier      – Classifies BLE devices by appearance / name / mfg
//   pax_store              – Central device database, persistence to SPIFFS
//   history                – Circular-buffer PAX history for graphing
//   display_manager        – Two-page touch UI (dashboard + detail list)
// ─────────────────────────────────────────────────────────────────────────────

#include <M5Unified.h>

#include "config.h"
#include "types.h"
#include "ble_scanner.h"
#include "wifi_scanner.h"
#include "pax_store.h"
#include "display_manager.h"
#include "history.h"

// ── Global history buffer ────────────────────────────────────────────────────
static PaxHistory g_history;
static unsigned long g_lastHistorySampleMs = 0;
static unsigned long g_lastDataProcessMs   = 0;
static unsigned long g_lastChannelHopMs    = 0;
static unsigned long g_lastCsvMs           = 0;

// ── Serial CSV output ────────────────────────────────────────────────────────
static void emitCsv() {
    Serial.printf("CSV,%lu,%d,%d,%d,%u\n",
                  millis(), getPaxTotal(), getPaxBle(), getPaxWifi(),
                  (unsigned)getDbSize());
}

// ── Arduino entry points ─────────────────────────────────────────────────────

void setup() {
    auto cfg = M5.config();
    M5.begin(cfg);
    Serial.begin(115200);
    Serial.println("\n=== PAX Counter M5Stack ===");

    initDisplay();
    loadClassifications();

    // BLE
    if (!initBle()) {
        M5.Lcd.fillScreen(TFT_RED);
        M5.Lcd.setTextSize(2);
        M5.Lcd.setCursor(10, 10);
        M5.Lcd.print("BLE Init FAILED");
        Serial.println("[FATAL] BLE init failed");
        while (true) delay(1000);
    }
    Serial.println("[BLE]  Ready");

    // WiFi probe-request sniffer (runs alongside BLE via ESP32 coexistence)
    initWifiScanner();

    // CSV header
    Serial.println("CSV,millis,pax_total,pax_ble,pax_wifi,db_size");

    // Initial render
    renderFrame(g_history);
}

void loop() {
    M5.update();
    unsigned long now = millis();

    // ── Touch handling ───────────────────────────────────────────────────
    handleTouch();

    // ── BLE scan (blocking for BLE_SCAN_TIME_S seconds) ──────────────────
    runBleScan();

    // ── WiFi: drain captured probe-request MACs into pax_store ───────────
    drainWifiCaptures();

    // ── WiFi channel hop ─────────────────────────────────────────────────
    if (now - g_lastChannelHopMs >= (unsigned long)WIFI_CHANNEL_HOP_MS) {
        hopWifiChannel();
        g_lastChannelHopMs = now;
    }

    // ── Process activity data (prune, count, sort) ───────────────────────
    if (now - g_lastDataProcessMs >= DATA_PROCESS_INTERVAL_MS) {
        processActivityData(currentFilterLabel());
        g_lastDataProcessMs = now;
    }

    // ── History sample ───────────────────────────────────────────────────
    if (now - g_lastHistorySampleMs >= HISTORY_SAMPLE_MS) {
        g_history.push(getPaxTotal());
        g_lastHistorySampleMs = now;
    }

    // ── Render ───────────────────────────────────────────────────────────
    renderFrame(g_history);

    // ── Periodic persistence ─────────────────────────────────────────────
    saveClassificationsIfNeeded();

    // ── Serial CSV output ────────────────────────────────────────────────
    if (now - g_lastCsvMs >= CSV_OUTPUT_INTERVAL_MS) {
        emitCsv();
        g_lastCsvMs = now;
    }

    delay(100);
}
