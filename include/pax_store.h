#pragma once
// ─────────────────────────────────────────────────────────────────────────────
// PAX Counter M5Stack - Device Database & Persistence
// ─────────────────────────────────────────────────────────────────────────────
// Central store for all detected devices.  Both BLE and WiFi scanners feed
// into paxStoreIngest().  The main loop calls processActivityData() to prune
// stale entries, recount PAX, and prepare sorted display lists.

#include "types.h"
#include <map>
#include <vector>
#include <string>

// ── Ingest a single sighting ─────────────────────────────────────────────────
// Called by ble_scanner and wifi_scanner.  `addTimestamp` should be true the
// first time a MAC is seen in a scan cycle to avoid duplicate timestamps.
void paxStoreIngest(uint64_t macKey, const std::string& macStr,
                    const std::string& label, int rssi,
                    ScanSource source, bool addTimestamp);

// ── Window processing ────────────────────────────────────────────────────────
// Prune old timestamps, recount PAX, rebuild sorted lists.
void processActivityData(const std::string& filterLabel);

// ── Persistence ──────────────────────────────────────────────────────────────
void loadClassifications();
void saveClassificationsIfNeeded();

// ── Accessors ────────────────────────────────────────────────────────────────
int  getPaxTotal();
int  getPaxBle();
int  getPaxWifi();
const std::vector<MacActivityInfo>& getTopCommon();
const std::vector<MacActivityInfo>& getRecentList();
size_t getDbSize();
