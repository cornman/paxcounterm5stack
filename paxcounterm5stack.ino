// ─────────────────────────────────────────────────────────────────────────────
// PAX Counter M5Stack
// ─────────────────────────────────────────────────────────────────────────────
// Counts nearby people by detecting BLE advertisements AND WiFi probe requests,
// then displays the count, a 60-minute history graph, and device detail lists on
// the M5Stack Core2 screen.
//
// This codebase is written to Holzmann's NASA/JPL "Power of Ten" rules: fixed
// static storage (no heap in steady state), statically-bounded loops, functions
// under ~60 lines, ≥2 assertions per function on average, and checked returns.
// See POWER_OF_TEN.md for the per-rule compliance report and declared waivers.
//
// Arduino IDE setup:
//   Board:     M5Stack Core2  (via ESP32 board package)
//   Libraries: M5Unified  (ArduinoJson is no longer required)
//
// Architecture (all files live in this sketch folder):
//   paxcounterm5stack.ino  – setup / loop orchestration (this file)
//   config.h               – all fixed capacities + tunables
//   classification.*       – Classification enum + static label/PAX tables
//   device_classifier.*    – classify a BLE advert into a Classification
//   ble_scanner.*          – ESP32 BLE scanning
//   wifi_scanner.*         – ESP32 WiFi promiscuous-mode probe-request capture
//   device_table.*         – fixed-capacity live device store (the DB)
//   known_cache.*          – fixed MAC→class LRU cache (survives reboot)
//   persistence.*          – fixed binary SPIFFS format
//   pax_store.*            – facade over the three above
//   history.h              – fixed circular-buffer PAX history
//   display_manager.*      – two-page touch UI (dashboard + detail list)
//   c_assert.h             – Power of Ten Rule 5 assertion macro
// ─────────────────────────────────────────────────────────────────────────────

#include <M5Unified.h>

#include "config.h"
#include "classification.h"
#include "ble_scanner.h"
#include "wifi_scanner.h"
#include "pax_store.h"
#include "display_manager.h"
#include "history.h"
#include "c_assert.h"

static PaxHistory    g_history;
static unsigned long g_lastHistoryMs = 0;
static unsigned long g_lastProcessMs = 0;
static unsigned long g_lastHopMs     = 0;
static unsigned long g_lastCsvMs     = 0;

// Emit one CSV telemetry row. printf's return is intentionally ignored (Rule 7:
// the failure action would equal the success action for a log line).
static void emitCsv(uint32_t now_ms) {
    (void)Serial.printf("CSV,%lu,%d,%d,%d,%u\n",
                        (unsigned long)now_ms, psPaxTotal(), psPaxBle(),
                        psPaxWifi(), (unsigned)psSize());
}

// Halt with a message on the LCD; used for unrecoverable init failures. The
// non-terminating loop is the Rule 2 exception (it must *not* terminate).
static void fatalHalt(const char* msg) {
    M5.Lcd.fillScreen(TFT_RED);
    M5.Lcd.setTextSize(2);
    M5.Lcd.setCursor(10, 10);
    M5.Lcd.print(msg);
    Serial.printf("[FATAL] %s\n", msg);
    for (;;) {
        delay(1000);
    }
}

void setup() {
    auto cfg = M5.config();
    M5.begin(cfg);
    Serial.begin(115200);
    Serial.println("\n=== PAX Counter M5Stack ===");

    initDisplay();
    psInit();

    if (!initBle()) {
        fatalHalt("BLE Init FAILED");
    }
    Serial.println("[BLE]  Ready");

    if (!initWifiScanner()) {
        // Non-fatal: BLE counting still works without the WiFi path.
        Serial.println("[WiFi] init failed (continuing BLE-only)");
    }

    Serial.println("CSV,millis,pax_total,pax_ble,pax_wifi,db_size");
    renderFrame(g_history);
}

void loop() {
    M5.update();
    uint32_t now = (uint32_t)millis();

    handleTouch();
    runBleScan(now);            // blocking for BLE_SCAN_TIME_S seconds
    drainWifiCaptures(now);

    if (now - g_lastHopMs >= (uint32_t)WIFI_CHANNEL_HOP_MS) {
        hopWifiChannel();
        g_lastHopMs = now;
    }
    if (now - g_lastProcessMs >= DATA_PROCESS_INTERVAL_MS) {
        psProcess(now, currentFilter());
        g_lastProcessMs = now;
    }
    if (now - g_lastHistoryMs >= HISTORY_SAMPLE_MS) {
        g_history.push(psPaxTotal());
        g_lastHistoryMs = now;
    }

    renderFrame(g_history);
    psSaveIfDue(now);

    if (now - g_lastCsvMs >= CSV_OUTPUT_INTERVAL_MS) {
        emitCsv(now);
        g_lastCsvMs = now;
    }
    delay(100);
}
