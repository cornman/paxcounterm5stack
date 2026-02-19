#pragma once
// ─────────────────────────────────────────────────────────────────────────────
// PAX Counter M5Stack - BLE Device Classifier
// ─────────────────────────────────────────────────────────────────────────────
// Classifies a BLE advertisement into one of the labels defined in types.h,
// using (in priority order): iBeacon structure, Eddystone UUID, manufacturer
// company ID, BLE appearance code, and device name keyword matching.

#include <BLEAdvertisedDevice.h>
#include <string>

// Returns one of the CLS_* labels from types.h.
std::string classifyDevice(BLEAdvertisedDevice& device);
