#pragma once
// ─────────────────────────────────────────────────────────────────────────────
// BLE Scanner
// ─────────────────────────────────────────────────────────────────────────────
// Wraps ESP32 BLE scanning. Call initBle() once in setup(); call runBleScan()
// each loop iteration. Discovered devices are classified and forwarded to the
// PAX store. `now_ms` (the caller's millis()) is passed in so timekeeping has a
// single source (Rule 6).

#include <cstdint>

bool initBle();                     // returns false on failure
void runBleScan(uint32_t now_ms);   // blocking scan for BLE_SCAN_TIME_S seconds
