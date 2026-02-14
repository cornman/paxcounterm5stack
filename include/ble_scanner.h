#pragma once
// ─────────────────────────────────────────────────────────────────────────────
// PAX Counter M5Stack - BLE Scanner
// ─────────────────────────────────────────────────────────────────────────────
// Wraps ESP32 BLE scanning.  Call initBle() once in setup(), then
// runBleScan() each loop iteration.  Discovered devices are fed directly
// into the PAX store.

#include <BLEScan.h>

bool initBle();                     // Returns false on failure
void runBleScan();                  // Blocking scan for BLE_SCAN_TIME_S seconds
