#pragma once
// ─────────────────────────────────────────────────────────────────────────────
// PAX Counter M5Stack - WiFi Probe Request Scanner
// ─────────────────────────────────────────────────────────────────────────────
// Passively captures 802.11 probe requests in promiscuous mode to detect
// nearby WiFi-enabled devices.  The promiscuous callback runs in the WiFi
// task context; collected MACs are drained into the PAX store from the main
// loop via drainWifiCaptures().
//
// Compile guard: everything is a no-op when ENABLE_WIFI_SCAN == 0.

#include "config.h"

#if ENABLE_WIFI_SCAN

void initWifiScanner();        // Call once in setup() AFTER initBle()
void hopWifiChannel();         // Call periodically to rotate channels
void drainWifiCaptures();      // Drain captured MACs into pax_store (main loop)

#else
inline void initWifiScanner()  {}
inline void hopWifiChannel()   {}
inline void drainWifiCaptures() {}
#endif
