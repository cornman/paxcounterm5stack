#pragma once
// ─────────────────────────────────────────────────────────────────────────────
// WiFi Probe Request Scanner
// ─────────────────────────────────────────────────────────────────────────────
// Passively captures 802.11 probe requests in promiscuous mode to detect nearby
// WiFi devices. The promiscuous callback runs in the WiFi task; captured MACs are
// drained into the PAX store from the main loop via drainWifiCaptures(now_ms).
//
// Rule 8 note: ENABLE_WIFI_SCAN is the project's ONE feature #ifdef (beyond
// include guards). It selects whether the WiFi radio is used at all; when 0,
// these become no-ops so the sketch builds and links without the WiFi path.

#include <cstdint>
#include "config.h"

#if ENABLE_WIFI_SCAN
bool initWifiScanner();                    // once in setup(); false on failure
void hopWifiChannel();                     // rotate channels periodically
void drainWifiCaptures(uint32_t now_ms);   // move captures into pax_store
#else
inline bool initWifiScanner()                 { return true; }
inline void hopWifiChannel()                  {}
inline void drainWifiCaptures(uint32_t)       {}
#endif
