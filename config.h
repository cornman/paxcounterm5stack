#pragma once
// ─────────────────────────────────────────────────────────────────────────────
// PAX Counter M5Stack - Configuration
// ─────────────────────────────────────────────────────────────────────────────
// All tuneable constants live here so the rest of the code stays clean.

#include <cstdint>
#include <cstddef>

// ── Fixed-capacity limits (Power of Ten Rule 3: no dynamic allocation) ───────
// Every runtime container in this project is a static array sized by one of
// these constants. They set a hard, provable ceiling on RAM. When a table is
// full the code drops new entries and logs it (no silent truncation) rather
// than allocating. Raise these to trade RAM for capacity.
static constexpr size_t DEV_MAX        = 256;  // max simultaneously-tracked devices
static constexpr size_t DEV_TS_RING    = 16;   // detection timestamps kept per device
static constexpr size_t MAC_STR_CAP    = 18;   // "aa:bb:cc:dd:ee:ff" + NUL
static constexpr size_t KNOWN_MAX      = 512;  // persistent MAC->class cache (LRU)
static constexpr size_t NAME_CAP       = 32;   // bounded copy of a BLE device name
static constexpr size_t MFG_CAP        = 32;   // bounded copy of BLE manufacturer data

// ── Scanning ─────────────────────────────────────────────────────────────────
static constexpr int    BLE_SCAN_TIME_S          = 5;       // BLE scan window (seconds)
static constexpr int    BLE_SCAN_INTERVAL        = 100;     // BLE scan interval (ms)
static constexpr int    BLE_SCAN_WINDOW          = 99;      // BLE scan window   (ms)

#ifndef ENABLE_WIFI_SCAN
#define ENABLE_WIFI_SCAN 1
#endif
static constexpr int    WIFI_CHANNEL_HOP_MS      = 300;     // Hop WiFi channel every N ms
static constexpr int    WIFI_NUM_CHANNELS        = 13;      // Channels 1–13
static constexpr size_t BLE_MAX_RESULTS          = 256;     // per-scan result cap (Rule 2)

// ── Activity window ──────────────────────────────────────────────────────────
static constexpr unsigned long WINDOW_DURATION_MS        = 60UL * 60 * 1000; // 1 hour
static constexpr unsigned long DATA_PROCESS_INTERVAL_MS  = 5000;

// ── Persistence ──────────────────────────────────────────────────────────────
// Fixed binary record format (no JSON, no dynamic parser). Each record is a
// uint64 MAC + uint8 class. See persistence.h for the on-flash layout.
static constexpr const char*   PERSISTENCE_FILE = "/pax_cls.bin";
static constexpr unsigned long SAVE_INTERVAL_MS = 15UL * 60 * 1000;
static constexpr uint32_t      PERSIST_MAGIC    = 0x50415831;  // "PAX1"
static constexpr uint8_t       PERSIST_VERSION  = 1;

// ── Display layout (320 x 240, landscape) ────────────────────────────────────
static constexpr int SCREEN_W = 320;
static constexpr int SCREEN_H = 240;

// PAX count area (top)
static constexpr int PAX_FONT_SZ     = 6;
static constexpr int PAX_FONT_H      = PAX_FONT_SZ * 8;
static constexpr int PAX_PAD         = 6;
static constexpr int PAX_REGION_H    = PAX_FONT_H + PAX_PAD;

// Detail / graph area
static constexpr int DETAIL_FONT_SZ  = 1;
static constexpr int DETAIL_FONT_H   = DETAIL_FONT_SZ * 8;
static constexpr int DETAIL_SPACING  = 2;
static constexpr int DETAIL_Y        = PAX_REGION_H;
static constexpr int DETAIL_H        = SCREEN_H - DETAIL_Y;
static constexpr int DETAIL_X        = 5;
static constexpr int LIST_INDENT     = 2;
static constexpr size_t CLS_DISPLAY_W = 9;

// Graph region (on dashboard page)
static constexpr int GRAPH_X         = 10;
static constexpr int GRAPH_Y         = PAX_REGION_H + 12;
static constexpr int GRAPH_W         = SCREEN_W - 20;
static constexpr int GRAPH_H         = 80;
static constexpr int GRAPH_BAR_W     = 4;
static constexpr int GRAPH_BAR_GAP   = 1;

// Display lists
static constexpr int TOP_N_COMMON    = 5;
static constexpr int RECENT_N        = 5;

// ── Touch button zones (M5Stack Core2 – bottom 40 px) ───────────────────────
static constexpr int TOUCH_Y_MIN = 200;
static constexpr int TOUCH_Y_MAX = 240;
static constexpr int TOUCH_A_XMIN = 0;    // Left
static constexpr int TOUCH_A_XMAX = 106;
static constexpr int TOUCH_B_XMIN = 107;  // Middle
static constexpr int TOUCH_B_XMAX = 212;
static constexpr int TOUCH_C_XMIN = 213;  // Right
static constexpr int TOUCH_C_XMAX = 319;

// ── History / graphing ───────────────────────────────────────────────────────
static constexpr int    HISTORY_MAX_ENTRIES   = 60;              // 60 samples
static constexpr unsigned long HISTORY_SAMPLE_MS = 60UL * 1000;  // 1 per minute

// ── Serial CSV output ────────────────────────────────────────────────────────
static constexpr unsigned long CSV_OUTPUT_INTERVAL_MS = 30000;   // Every 30 s

// ── BLE Appearance Codes ─────────────────────────────────────────────────────
static constexpr uint16_t APP_UNKNOWN         = 0x0000;
static constexpr uint16_t APP_PHONE           = 0x0240;
static constexpr uint16_t APP_COMPUTER        = 0x0080;
static constexpr uint16_t APP_WATCH           = 0x00C0;
static constexpr uint16_t APP_SMARTWATCH      = 0x0241;
static constexpr uint16_t APP_TABLET          = 0x0180;
static constexpr uint16_t APP_HEADSET         = 0x0941;
static constexpr uint16_t APP_HANDSFREE       = 0x0942;
static constexpr uint16_t APP_EARBUD          = 0x0947;
static constexpr uint16_t APP_TAG             = 0x0500;
static constexpr uint16_t APP_THERMO          = 0x0300;
static constexpr uint16_t APP_HR_SENSOR       = 0x0340;
static constexpr uint16_t APP_CYCLING_COMP    = 0x0483;
static constexpr uint16_t APP_HID             = 0x03C0;

// ── Bluetooth Company IDs ────────────────────────────────────────────────────
static constexpr uint16_t CID_APPLE     = 0x004C;
static constexpr uint16_t CID_MICROSOFT = 0x0006;
static constexpr uint16_t CID_SAMSUNG   = 0x0075;
static constexpr uint16_t CID_GOOGLE    = 0x00E0;

// ── iBeacon constants ────────────────────────────────────────────────────────
static constexpr uint8_t IBEACON_TYPE   = 0x02;
static constexpr uint8_t IBEACON_LEN    = 0x15;
