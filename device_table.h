#pragma once
// ─────────────────────────────────────────────────────────────────────────────
// Device Table — fixed-capacity live device store
// ─────────────────────────────────────────────────────────────────────────────
// Replaces the old std::map<uint64_t, MacActivityInfo> + per-device std::deque +
// std::string + std::vector. Everything here is static storage sized by DEV_MAX /
// DEV_TS_RING (Power of Ten Rule 3). All loops are bounded by those constants
// (Rule 2). `now_ms` is passed in rather than read from millis() so the whole
// module is pure and unit-testable on the host, and so time has the smallest
// possible scope (Rule 6).
//
// Capacity policy (Rule 3): when the table is full, a *new* device is dropped and
// counted in dtDroppedInserts() — never allocated. Existing devices still update.

#include <cstdint>
#include <cstddef>
#include "config.h"
#include "types.h"

// Compact, display-facing snapshot of one device. Contains no pointers into the
// table's internal state, so the UI cannot corrupt the store (Rule 6).
struct DeviceView {
    uint64_t mac_key;        // stable identity + deterministic tie-break
    char     mac_str[MAC_STR_CAP];
    uint8_t  cls;            // Classification enum value
    int16_t  rssi;
    uint8_t  source;         // ScanSource
    uint16_t count;          // detections in window (saturates at DEV_TS_RING)
    uint32_t last_seen_ms;
    uint32_t age_ms;         // now - last_seen at selection time (wrap-safe)
};

// Reset the table to empty. Call once at startup (Rule 3: allocation happens
// only via static storage, this just clears it).
void dtInit();

// Record one sighting of `mac_key`.
//   mac_str       : pre-formatted "aa:bb:.." (copied, bounded to MAC_STR_CAP)
//   cls           : Classification enum; CLS_UNKNOWN/CLS_OTHER_BLE are "weak"
//   rssi, source  : latest signal + which radio
//   add_timestamp : true once per scan cycle per MAC (dedupes within a cycle)
//   now_ms        : caller's millis()
// Returns false if a *new* device was dropped because the table is full, or on
// invalid arguments; true otherwise.
bool dtIngest(uint64_t mac_key, const char* mac_str, uint8_t cls,
              int rssi, uint8_t source, bool add_timestamp, uint32_t now_ms);

// Prune detections older than `window_ms`, drop devices with none left, recompute
// PAX totals, and rebuild the top-common / recent selection lists filtered by
// `filter` (a Classification value, or FILTER_ALL for no filter).
void dtProcess(uint32_t now_ms, uint32_t window_ms, uint8_t filter);

// ── Results (valid after dtProcess) ──────────────────────────────────────────
int    dtPaxTotal();          // devices in window whose class counts as PAX
int    dtPaxBle();            // ...seen on BLE
int    dtPaxWifi();           // ...seen on WiFi
size_t dtSize();              // total devices currently tracked (any class)
uint32_t dtDroppedInserts();  // cumulative new-device drops due to a full table

// Selection lists. Return a pointer to a static array and write its length to
// *count (0..TOP_N_COMMON / RECENT_N). Never returns NULL.
const DeviceView* dtTopCommon(size_t* count);
const DeviceView* dtRecent(size_t* count);
