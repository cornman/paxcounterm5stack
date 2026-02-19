// ─────────────────────────────────────────────────────────────────────────────
// PAX Counter M5Stack - BLE Device Classifier
// ─────────────────────────────────────────────────────────────────────────────
#include "device_classifier.h"
#include "config.h"
#include "types.h"

#include <algorithm>
#include <cctype>
#include <string>

// ── Helpers ──────────────────────────────────────────────────────────────────
static std::string toLower(const std::string& s) {
    std::string out = s;
    std::transform(out.begin(), out.end(), out.begin(),
                   [](unsigned char c) { return std::tolower(c); });
    return out;
}

static bool contains(const std::string& haystack, const char* needle) {
    return haystack.find(needle) != std::string::npos;
}

// ── Main classification logic ────────────────────────────────────────────────
std::string classifyDevice(BLEAdvertisedDevice& device) {

    // 1) iBeacon — Apple manufacturer data with beacon header
    if (device.haveManufacturerData()) {
        std::string mfg = device.getManufacturerData();
        if (mfg.length() >= 4) {
            uint16_t cid = ((uint8_t)mfg[1] << 8) | (uint8_t)mfg[0];
            if (cid == CID_APPLE &&
                (uint8_t)mfg[2] == IBEACON_TYPE &&
                (uint8_t)mfg[3] == IBEACON_LEN) {
                return CLS_IBEACON;
            }
        }
    }

    // 2) Eddystone beacon
    if (device.haveServiceUUID() &&
        device.isAdvertisingService(BLEUUID((uint16_t)0xFEAA))) {
        return CLS_EDDYSTONE;
    }

    // 3) Manufacturer company ID (non-Apple)
    if (device.haveManufacturerData()) {
        std::string mfg = device.getManufacturerData();
        if (mfg.length() >= 2) {
            uint16_t cid = ((uint8_t)mfg[1] << 8) | (uint8_t)mfg[0];
            switch (cid) {
                case CID_MICROSOFT: return CLS_MSFT_DEV;
                case CID_SAMSUNG:   return CLS_SAMSUNG_DEV;
                case CID_GOOGLE:    return CLS_GOOGLE_DEV;
            }
        }
    }

    // 4) BLE appearance code
    if (device.haveAppearance()) {
        switch (device.getAppearance()) {
            case APP_PHONE:                             return CLS_PHONE;
            case APP_WATCH:      case APP_SMARTWATCH:   return CLS_WATCH;
            case APP_COMPUTER:                          return CLS_COMPUTER;
            case APP_TABLET:                            return CLS_TABLET;
            case APP_HEADSET:    case APP_HANDSFREE:
            case APP_EARBUD:                            return CLS_AUDIO;
            case APP_TAG:                               return CLS_TAG;
            case APP_THERMO:     case APP_HR_SENSOR:
            case APP_CYCLING_COMP:                      return CLS_SENSOR;
            case APP_HID:                               return CLS_HID;
        }
    }

    // 5) Name-based heuristics (only when appearance is absent)
    if (device.haveName()) {
        std::string name = toLower(device.getName());

        if (contains(name, "meshtastic")) return CLS_MESHTASTIC;

        if (!device.haveAppearance()) {
            if (contains(name, "beacon"))                           return CLS_IBEACON;
            if (contains(name, "sensor"))                           return CLS_SENSOR;
            if (contains(name, "tag") || contains(name, "tile"))    return CLS_TAG;
            if (contains(name, "phone"))                            return CLS_PHONE;
            if (contains(name, "galaxy watch"))                     return CLS_WATCH;
            if (contains(name, "watch"))                            return CLS_WATCH;
            if (contains(name, "airpods") ||
                contains(name, "galaxy buds") ||
                contains(name, "pixel buds") ||
                contains(name, "headset") ||
                contains(name, "earbuds"))                          return CLS_AUDIO;
            if (contains(name, "surface")) {
                if (contains(name, "headphones")) return CLS_AUDIO;
                return CLS_MSFT_DEV;
            }
        }
    }

    // 6) Has *some* advertising data → generic BLE device
    if (device.haveName() || device.haveServiceUUID() ||
        device.haveManufacturerData() || device.haveAppearance()) {
        return CLS_OTHER_BLE;
    }

    return CLS_UNKNOWN;
}
