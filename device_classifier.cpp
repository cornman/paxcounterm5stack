// ─────────────────────────────────────────────────────────────────────────────
// BLE Device Classifier
// ─────────────────────────────────────────────────────────────────────────────
#include "device_classifier.h"
#include "config.h"
#include "classification.h"
#include "c_assert.h"

#include <cctype>
#include <cstring>
#include <string>       // library boundary only (see copyMfg / copyLowerName)

// CLS_COUNT is used here as an internal "no decision" sentinel returned by the
// per-strategy helpers; classifyDevice() never returns it to callers.

// ── Copy advertisement data into fixed buffers (the library boundary) ────────
// getManufacturerData()/getName() return library-owned std::strings; we copy the
// bytes we need into static-size buffers and let the temporaries die. Downstream
// logic touches only fixed buffers (Rules 2, 3).
static size_t copyMfg(BLEAdvertisedDevice& device, uint8_t* dst, size_t cap) {
    if (!c_assert(dst != nullptr) || !c_assert(cap > 0)) {
        return 0;
    }
    std::string data = device.getManufacturerData();
    size_t n = data.length();
    if (n > cap) {
        n = cap;
    }
    for (size_t i = 0; i < n; ++i) {                 // bound: cap (Rule 2)
        dst[i] = (uint8_t)data[i];
    }
    return n;
}

static void copyLowerName(BLEAdvertisedDevice& device, char* dst, size_t cap) {
    if (!c_assert(dst != nullptr) || !c_assert(cap > 1)) {
        return;
    }
    std::string name = device.getName();
    size_t n = name.length();
    if (n > cap - 1) {
        n = cap - 1;
    }
    for (size_t i = 0; i < n; ++i) {                 // bound: cap (Rule 2)
        dst[i] = (char)std::tolower((unsigned char)name[i]);
    }
    dst[n] = '\0';
}

static uint16_t companyId(const uint8_t* mfg, size_t n) {
    if (!c_assert(mfg != nullptr) || n < 2) {
        return 0;
    }
    return (uint16_t)(((uint16_t)mfg[1] << 8) | mfg[0]);   // little-endian
}

// ── Per-strategy classifiers (return CLS_COUNT when they cannot decide) ──────
static uint8_t classifyBeacon(const uint8_t* mfg, size_t n) {
    if (!c_assert(mfg != nullptr)) {
        return CLS_COUNT;
    }
    if (n >= 4 && companyId(mfg, n) == CID_APPLE &&
        mfg[2] == IBEACON_TYPE && mfg[3] == IBEACON_LEN) {
        return CLS_IBEACON;
    }
    return CLS_COUNT;
}

static uint8_t classifyByManufacturer(const uint8_t* mfg, size_t n) {
    if (!c_assert(mfg != nullptr) || n < 2) {
        return CLS_COUNT;
    }
    switch (companyId(mfg, n)) {
        case CID_MICROSOFT: return CLS_MSFT;
        case CID_SAMSUNG:   return CLS_SAMSUNG;
        case CID_GOOGLE:    return CLS_GOOGLE;
        default:            return CLS_COUNT;
    }
}

static uint8_t classifyByAppearance(uint16_t appearance) {
    switch (appearance) {
        case APP_PHONE:                          return CLS_PHONE;
        case APP_WATCH: case APP_SMARTWATCH:     return CLS_WATCH;
        case APP_COMPUTER:                       return CLS_COMPUTER;
        case APP_TABLET:                         return CLS_TABLET;
        case APP_HEADSET: case APP_HANDSFREE:
        case APP_EARBUD:                         return CLS_AUDIO;
        case APP_TAG:                            return CLS_TAG;
        case APP_THERMO: case APP_HR_SENSOR:
        case APP_CYCLING_COMP:                   return CLS_SENSOR;
        case APP_HID:                            return CLS_HID;
        default:                                 return CLS_COUNT;
    }
}

static bool has(const char* hay, const char* needle) {
    if (!c_assert(hay != nullptr) || !c_assert(needle != nullptr)) {
        return false;
    }
    return std::strstr(hay, needle) != nullptr;      // bounded by NAME_CAP input
}

// `lname` is already lower-cased. `have_appearance` gates the generic keyword
// heuristics (skipped when a precise appearance code was present but unmatched).
static uint8_t classifyByName(const char* lname, bool have_appearance) {
    if (!c_assert(lname != nullptr)) {
        return CLS_COUNT;
    }
    if (has(lname, "meshtastic")) {
        return CLS_MESHTASTIC;
    }
    if (have_appearance) {
        return CLS_COUNT;
    }
    if (has(lname, "beacon"))                              return CLS_IBEACON;
    if (has(lname, "sensor"))                              return CLS_SENSOR;
    if (has(lname, "tag") || has(lname, "tile"))          return CLS_TAG;
    if (has(lname, "phone"))                              return CLS_PHONE;
    if (has(lname, "galaxy watch") || has(lname, "watch")) return CLS_WATCH;
    if (has(lname, "airpods") || has(lname, "galaxy buds") ||
        has(lname, "pixel buds") || has(lname, "headset") ||
        has(lname, "earbuds"))                            return CLS_AUDIO;
    if (has(lname, "surface")) {
        return has(lname, "headphones") ? CLS_AUDIO : CLS_MSFT;
    }
    return CLS_COUNT;
}

// ── Orchestration ────────────────────────────────────────────────────────────
uint8_t classifyDevice(BLEAdvertisedDevice& device) {
    uint8_t  mfg[MFG_CAP];
    size_t   mfg_n   = 0;
    bool     has_mfg = device.haveManufacturerData();
    if (has_mfg) {
        mfg_n = copyMfg(device, mfg, MFG_CAP);
    }

    uint8_t c;
    if (has_mfg) {
        c = classifyBeacon(mfg, mfg_n);
        if (c != CLS_COUNT) {
            return c;
        }
    }
    if (device.haveServiceUUID() &&
        device.isAdvertisingService(BLEUUID((uint16_t)0xFEAA))) {
        return CLS_EDDYSTONE;
    }
    if (has_mfg) {
        c = classifyByManufacturer(mfg, mfg_n);
        if (c != CLS_COUNT) {
            return c;
        }
    }
    bool has_app = device.haveAppearance();
    if (has_app) {
        c = classifyByAppearance(device.getAppearance());
        if (c != CLS_COUNT) {
            return c;
        }
    }
    if (device.haveName()) {
        char lname[NAME_CAP];
        copyLowerName(device, lname, NAME_CAP);
        c = classifyByName(lname, has_app);
        if (c != CLS_COUNT) {
            return c;
        }
    }
    if (device.haveName() || device.haveServiceUUID() || has_mfg || has_app) {
        return CLS_OTHER_BLE;
    }
    return CLS_UNKNOWN;
}
