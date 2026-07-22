// ─────────────────────────────────────────────────────────────────────────────
// Device classification — static tables
// ─────────────────────────────────────────────────────────────────────────────
#include "classification.h"
#include "c_assert.h"

// Label table. One entry per Classification value; index with the enum. Kept to
// <= 9 characters so the detail list fits the 320-px screen.
static const char* const LABELS[CLS_COUNT] = {
    "Unknown",    // CLS_UNKNOWN
    "Phone",      // CLS_PHONE
    "Watch",      // CLS_WATCH
    "Audio",      // CLS_AUDIO
    "PC/Lap",     // CLS_COMPUTER
    "Tablet",     // CLS_TABLET
    "Tag",        // CLS_TAG
    "iBeacon",    // CLS_IBEACON
    "Eddystone",  // CLS_EDDYSTONE
    "Sensor",     // CLS_SENSOR
    "Meshtast",   // CLS_MESHTASTIC
    "HID",        // CLS_HID
    "OtherBLE",   // CLS_OTHER_BLE
    "Samsung",    // CLS_SAMSUNG
    "Google",     // CLS_GOOGLE
    "MSFT",       // CLS_MSFT
    "WiFi",       // CLS_WIFI
};

// PAX-membership table: does this class represent a person-carried device?
// Beacons, sensors, tags, HID and infrastructure are excluded.
static const bool IS_PAX[CLS_COUNT] = {
    true,   // CLS_UNKNOWN    — count; most unknowns are phones/wearables
    true,   // CLS_PHONE
    true,   // CLS_WATCH
    true,   // CLS_AUDIO
    true,   // CLS_COMPUTER
    true,   // CLS_TABLET
    false,  // CLS_TAG
    false,  // CLS_IBEACON
    false,  // CLS_EDDYSTONE
    false,  // CLS_SENSOR
    false,  // CLS_MESHTASTIC
    false,  // CLS_HID
    true,   // CLS_OTHER_BLE
    true,   // CLS_SAMSUNG
    true,   // CLS_GOOGLE
    true,   // CLS_MSFT
    true,   // CLS_WIFI
};

bool classIsValid(uint8_t c) {
    return c < CLS_COUNT;
}

const char* classLabel(uint8_t c) {
    if (!c_assert(classIsValid(c))) {
        return "?";
    }
    return LABELS[c];
}

bool classIsPax(uint8_t c) {
    if (!c_assert(classIsValid(c))) {
        return false;
    }
    return IS_PAX[c];
}

bool classIsSpecific(uint8_t c) {
    return classIsValid(c) && c != CLS_UNKNOWN && c != CLS_OTHER_BLE;
}
