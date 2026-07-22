#pragma once
// ─────────────────────────────────────────────────────────────────────────────
// BLE Device Classifier
// ─────────────────────────────────────────────────────────────────────────────
// Classifies a BLE advertisement into a Classification enum value (see
// classification.h), using (in priority order): iBeacon structure, Eddystone
// UUID, manufacturer company ID, BLE appearance code, and device-name keywords.
//
// Target-only (depends on the BLE stack). All advertisement data is copied into
// small fixed buffers before inspection, so no dynamic data flows into the logic.

#include <BLEAdvertisedDevice.h>
#include <cstdint>

// Returns a Classification enum value (never CLS_COUNT).
uint8_t classifyDevice(BLEAdvertisedDevice& device);
