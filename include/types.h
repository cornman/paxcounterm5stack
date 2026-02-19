#pragma once
// ─────────────────────────────────────────────────────────────────────────────
// PAX Counter M5Stack - Shared Types & Utilities
// ─────────────────────────────────────────────────────────────────────────────
// Pure C++ – no Arduino or ESP-IDF dependencies – so these are testable
// natively on the host.

#include <string>
#include <deque>
#include <set>

// ── Scan source tag ──────────────────────────────────────────────────────────
enum class ScanSource : uint8_t { BLE, WIFI };

// ── Classification labels ────────────────────────────────────────────────────
// Short strings that fit the 320-px display.  Kept as inline constants so
// every translation unit that includes this header sees the same objects.
inline const std::string CLS_PHONE       = "Phone";
inline const std::string CLS_WATCH       = "Watch";
inline const std::string CLS_AUDIO       = "Audio";
inline const std::string CLS_COMPUTER    = "PC/Lap";
inline const std::string CLS_TABLET      = "Tablet";
inline const std::string CLS_TAG         = "Tag";
inline const std::string CLS_IBEACON     = "iBeacon";
inline const std::string CLS_EDDYSTONE   = "Eddystone";
inline const std::string CLS_SENSOR      = "Sensor";
inline const std::string CLS_MESHTASTIC  = "Meshtastic";
inline const std::string CLS_HID         = "HID";
inline const std::string CLS_OTHER_BLE   = "OtherBLE";
inline const std::string CLS_UNKNOWN     = "Unknown";
inline const std::string CLS_SAMSUNG_DEV = "Samsung";
inline const std::string CLS_GOOGLE_DEV  = "Google";
inline const std::string CLS_MSFT_DEV    = "MSFT";
inline const std::string CLS_WIFI_PROBE  = "WiFi";

// ── PAX-relevant set ─────────────────────────────────────────────────────────
// These classification labels count toward the PAX total.
inline const std::set<std::string> PAX_RELEVANT = {
    CLS_PHONE, CLS_WATCH, CLS_AUDIO, CLS_COMPUTER, CLS_TABLET,
    CLS_OTHER_BLE, CLS_UNKNOWN, CLS_SAMSUNG_DEV, CLS_GOOGLE_DEV,
    CLS_MSFT_DEV, CLS_WIFI_PROBE,
};

// ── Per-device tracking record ───────────────────────────────────────────────
struct MacActivityInfo {
    std::string  mac_str;                    // Human-readable "aa:bb:cc:dd:ee:ff"
    uint64_t     mac_key         = 0;
    std::deque<unsigned long> timestamps;    // Detection times within the window
    size_t       count_in_window = 0;
    std::string  classification  = "Unknown";
    int          rssi            = 0;
    ScanSource   source          = ScanSource::BLE;

    MacActivityInfo() = default;
    MacActivityInfo(uint64_t key, const std::string& mac, ScanSource src)
        : mac_str(mac), mac_key(key), source(src) {}

    // Default sort: most detections first, then by MAC for stability.
    bool operator<(const MacActivityInfo& o) const {
        if (count_in_window != o.count_in_window)
            return count_in_window > o.count_in_window;
        return mac_key < o.mac_key;
    }
};

// ── Recency comparator ──────────────────────────────────────────────────────
struct ByRecency {
    bool operator()(const MacActivityInfo& a, const MacActivityInfo& b) const {
        unsigned long ta = a.timestamps.empty() ? 0 : a.timestamps.back();
        unsigned long tb = b.timestamps.empty() ? 0 : b.timestamps.back();
        if (ta != tb) return ta > tb;           // Newest first
        return a.mac_key < b.mac_key;
    }
};

// ── Pure utility functions ───────────────────────────────────────────────────

// Truncate `str` to at most `width` characters, appending "." if truncated.
inline std::string truncateString(const std::string& str, size_t width) {
    if (str.length() <= width) return str;
    if (width > 1) return str.substr(0, width - 1) + ".";
    if (width == 1) return str.substr(0, 1);
    return "";
}

// Format a MAC uint64_t (48-bit) into "aa:bb:cc:dd:ee:ff".
inline std::string macToString(uint64_t mac) {
    char buf[18];
    snprintf(buf, sizeof(buf), "%02x:%02x:%02x:%02x:%02x:%02x",
             (unsigned)((mac >> 40) & 0xFF), (unsigned)((mac >> 32) & 0xFF),
             (unsigned)((mac >> 24) & 0xFF), (unsigned)((mac >> 16) & 0xFF),
             (unsigned)((mac >>  8) & 0xFF), (unsigned)(mac & 0xFF));
    return buf;
}
