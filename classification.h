#pragma once
// ─────────────────────────────────────────────────────────────────────────────
// Device classification — fixed enum + static tables
// ─────────────────────────────────────────────────────────────────────────────
// Replaces the previous std::string labels and std::set<std::string> membership
// test. A device's class is a single uint8_t; labels and "counts as PAX?" are
// looked up in static const arrays indexed by that enum. No heap, no string
// comparison, O(1) membership (Power of Ten Rules 3, 6, 9).
//
// Pure C++: no Arduino / ESP-IDF dependency, so it is unit-tested natively.

#include <cstdint>

// Order is stable and persisted on flash as a raw byte — only APPEND new classes
// at the end (before CLS_COUNT), never reorder or remove.
enum Classification : uint8_t {
    CLS_UNKNOWN = 0,
    CLS_PHONE,
    CLS_WATCH,
    CLS_AUDIO,
    CLS_COMPUTER,
    CLS_TABLET,
    CLS_TAG,
    CLS_IBEACON,
    CLS_EDDYSTONE,
    CLS_SENSOR,
    CLS_MESHTASTIC,
    CLS_HID,
    CLS_OTHER_BLE,
    CLS_SAMSUNG,
    CLS_GOOGLE,
    CLS_MSFT,
    CLS_WIFI,
    CLS_COUNT            // sentinel: number of real classes; never a valid class
};

// Sentinel filter value meaning "match every class" (used by the UI filter).
static constexpr uint8_t FILTER_ALL = CLS_COUNT;

// True if `c` is a valid Classification (in range). Rule 7 parameter validation.
bool classIsValid(uint8_t c);

// Short display label for class `c` (<= 9 chars). Returns "?" if `c` is invalid,
// never NULL — callers may print the result directly.
const char* classLabel(uint8_t c);

// True if class `c` counts toward the PAX total (people-carried devices), false
// for beacons / sensors / tags / infrastructure. Returns false if `c` invalid.
bool classIsPax(uint8_t c);

// True if `c` is a *specific* class — valid and neither CLS_UNKNOWN nor
// CLS_OTHER_BLE. Only specific classes are worth caching/persisting or using to
// upgrade an existing record.
bool classIsSpecific(uint8_t c);
