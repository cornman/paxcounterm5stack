// ─────────────────────────────────────────────────────────────────────────────
// PAX Store — facade over the device table, known cache, and persistence
// ─────────────────────────────────────────────────────────────────────────────
#include "pax_store.h"
#include "config.h"
#include "classification.h"
#include "known_cache.h"
#include "persistence.h"
#include "device_table.h"
#include "c_assert.h"

#ifdef ARDUINO
#include <Arduino.h>       // Serial for status logging
#endif

static uint32_t g_last_save_ms = 0;

void psInit() {
    dtInit();
    kcInit();
#ifdef ARDUINO
    if (!persistLoad()) {
        Serial.println("[Store] persistLoad failed (starting empty)");
    }
#endif
    g_last_save_ms = 0;
}

bool psIngest(uint64_t mac_key, const char* mac_str, uint8_t cls,
              int rssi, uint8_t source, bool add_timestamp, uint32_t now_ms) {
    if (!c_assert(mac_str != nullptr)) {
        return false;
    }
    if (!c_assert(classIsValid(cls))) {
        cls = CLS_UNKNOWN;
    }
    // Upgrade a weak class from the persistent cache if we have seen this MAC.
    uint8_t resolved = cls;
    if (!classIsSpecific(cls)) {
        bool found = false;
        uint8_t known = kcLookup(mac_key, &found);
        if (found) {
            resolved = known;
        }
    }
    bool ok = dtIngest(mac_key, mac_str, resolved, rssi, source,
                       add_timestamp, now_ms);
    // Remember a specific class for next time (survives window-expiry + reboot).
    if (classIsSpecific(resolved)) {
        (void)kcUpdate(mac_key, resolved);
    }
    return ok;
}

void psProcess(uint32_t now_ms, uint8_t filter) {
    if (!c_assert(WINDOW_DURATION_MS <= 0xFFFFFFFFUL)) {
        return;
    }
    dtProcess(now_ms, (uint32_t)WINDOW_DURATION_MS, filter);
}

void psSaveIfDue(uint32_t now_ms) {
    if (!kcDirty()) {
        return;
    }
    if ((now_ms - g_last_save_ms) < SAVE_INTERVAL_MS) {
        return;
    }
#ifdef ARDUINO
    if (persistSave()) {
        kcClearDirty();
        g_last_save_ms = now_ms;
        Serial.printf("[Store] Saved %u classifications\n", (unsigned)kcSize());
    } else {
        Serial.println("[Store] persistSave failed");
    }
#else
    kcClearDirty();
    g_last_save_ms = now_ms;
#endif
}

int      psPaxTotal() { return dtPaxTotal(); }
int      psPaxBle()   { return dtPaxBle(); }
int      psPaxWifi()  { return dtPaxWifi(); }
size_t   psSize()     { return dtSize(); }
uint32_t psDropped()  { return dtDroppedInserts(); }

const DeviceView* psTopCommon(size_t* count) {
    (void)c_assert(count != nullptr);
    return dtTopCommon(count);
}
const DeviceView* psRecent(size_t* count) {
    (void)c_assert(count != nullptr);
    return dtRecent(count);
}
