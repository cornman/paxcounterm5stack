// ─────────────────────────────────────────────────────────────────────────────
// PAX Counter M5Stack — Native Unit Tests
// ─────────────────────────────────────────────────────────────────────────────
// Exercises the pure, Arduino-free core: string helpers, history, classification
// tables, the fixed-capacity device table, the known-classification cache, and
// binary persistence. Build & run from the sketch root with run_tests.sh, or:
//
//   g++ -std=c++17 -Wall -Wextra -Wpedantic -I . (all *.cpp core files)
//       test/test_native/test_helpers.cpp -o test_pax && ./test_pax

#include <cstdio>
#include <cstdint>
#include <cstring>
#include <string>

#include "types.h"
#include "history.h"
#include "classification.h"
#include "device_table.h"
#include "known_cache.h"
#include "persistence.h"
#include "config.h"

// ── Minimal test framework (no external deps) ────────────────────────────────
static int g_tests  = 0;
static int g_passed = 0;

#define TEST(name)                                         \
    static void test_##name();                             \
    static struct Register_##name {                        \
        Register_##name() { test_##name(); }               \
    } reg_##name;                                          \
    static void test_##name()

#define CHECK(cond)                                        \
    do {                                                   \
        g_tests++;                                         \
        if (cond) { g_passed++; }                          \
        else { printf("  FAIL %s:%d  %s\n",                \
                      __FILE__, __LINE__, #cond); }        \
    } while (0)

// ── String helpers ───────────────────────────────────────────────────────────
TEST(mac_to_str_basic) {
    char b[MAC_STR_CAP];
    CHECK(macToStr(b, sizeof(b), 0xAABBCCDDEEFFULL));
    CHECK(std::string(b) == "aa:bb:cc:dd:ee:ff");
}
TEST(mac_to_str_zero) {
    char b[MAC_STR_CAP];
    CHECK(macToStr(b, sizeof(b), 0));
    CHECK(std::string(b) == "00:00:00:00:00:00");
}
TEST(mac_to_str_small_buffer_rejected) {
    char b[8] = {1};
    CHECK(!macToStr(b, sizeof(b), 0x112233445566ULL));
    CHECK(b[0] == '\0');
}
TEST(truncate_shorter_unchanged) {
    char b[16];
    CHECK(truncateStr(b, sizeof(b), "abc", 5));
    CHECK(std::string(b) == "abc");
}
TEST(truncate_exact_fit) {
    char b[16];
    CHECK(truncateStr(b, sizeof(b), "abcde", 5));
    CHECK(std::string(b) == "abcde");
}
TEST(truncate_longer_gets_dot) {
    char b[16];
    CHECK(truncateStr(b, sizeof(b), "abcdef", 5));
    CHECK(std::string(b) == "abcd.");
}
TEST(truncate_width_one) {
    char b[16];
    CHECK(truncateStr(b, sizeof(b), "hello", 1));
    CHECK(std::string(b) == "h");
}
TEST(truncate_respects_dest_cap) {
    char b[4];  // room for 3 chars + NUL
    CHECK(truncateStr(b, sizeof(b), "abcdefgh", 6));
    CHECK(std::strlen(b) <= 3);
}
TEST(bounded_len_stops_at_cap) {
    CHECK(boundedLen("abcdef", 3) == 3);
    CHECK(boundedLen("ab", 8) == 2);
}

// ── History ───────────────────────────────────────────────────────────────────
TEST(history_empty) {
    PaxHistory h;
    CHECK(h.count == 0);
    CHECK(h.newest() == 0);
    CHECK(h.windowMax() == 0);
    CHECK(h.average() == 0L);
}
TEST(history_push_read_peak) {
    PaxHistory h;
    h.push(10); h.push(20); h.push(30);
    CHECK(h.count == 3);
    CHECK(h.at(0) == 10);
    CHECK(h.newest() == 30);
    CHECK(h.windowMax() == 30);
    CHECK(h.average() == 20L);
    h.push(5);
    CHECK(h.peak == 30);
}
TEST(history_wraps) {
    PaxHistory h;
    for (int i = 0; i < PaxHistory::CAPACITY + 10; i++) h.push(i);
    CHECK(h.count == PaxHistory::CAPACITY);
    CHECK(h.at(0) == 10);
    CHECK(h.newest() == PaxHistory::CAPACITY + 9);
}
TEST(history_bad_index_returns_zero) {
    PaxHistory h;
    h.push(7);
    CHECK(h.at(5) == 0);   // out of range -> 0, no OOB read
}

// ── Classification tables ─────────────────────────────────────────────────────
TEST(class_valid_range) {
    CHECK(classIsValid(CLS_PHONE));
    CHECK(classIsValid(CLS_WIFI));
    CHECK(!classIsValid(CLS_COUNT));
    CHECK(!classIsValid(200));
}
TEST(class_labels_nonnull) {
    for (uint8_t c = 0; c < CLS_COUNT; ++c) {
        CHECK(classLabel(c) != nullptr);
        CHECK(classLabel(c)[0] != '\0');
    }
    CHECK(std::string(classLabel(CLS_PHONE)) == "Phone");
    CHECK(std::string(classLabel(200)) == "?");
}
TEST(class_pax_membership) {
    CHECK(classIsPax(CLS_PHONE));
    CHECK(classIsPax(CLS_WIFI));
    CHECK(classIsPax(CLS_UNKNOWN));
    CHECK(!classIsPax(CLS_IBEACON));
    CHECK(!classIsPax(CLS_EDDYSTONE));
    CHECK(!classIsPax(CLS_SENSOR));
    CHECK(!classIsPax(CLS_TAG));
}
TEST(class_specific) {
    CHECK(classIsSpecific(CLS_PHONE));
    CHECK(classIsSpecific(CLS_WIFI));
    CHECK(!classIsSpecific(CLS_UNKNOWN));
    CHECK(!classIsSpecific(CLS_OTHER_BLE));
    CHECK(!classIsSpecific(200));
}

// ── Device table ──────────────────────────────────────────────────────────────
static void ingestN(uint64_t mac, uint8_t cls, uint8_t src, int times, uint32_t t) {
    char s[MAC_STR_CAP];
    macToStr(s, sizeof(s), mac);
    for (int i = 0; i < times; ++i) {
        dtIngest(mac, s, cls, -50, src, true, t);
    }
}
TEST(dt_counts_pax) {
    dtInit();
    ingestN(0x1, CLS_PHONE,   SRC_BLE,  1, 1000);
    ingestN(0x2, CLS_WIFI,    SRC_WIFI, 1, 1000);
    ingestN(0x3, CLS_IBEACON, SRC_BLE,  1, 1000);   // not PAX
    dtProcess(1000, WINDOW_DURATION_MS, FILTER_ALL);
    CHECK(dtSize() == 3);
    CHECK(dtPaxTotal() == 2);
    CHECK(dtPaxBle() == 1);
    CHECK(dtPaxWifi() == 1);
}
TEST(dt_prunes_stale) {
    dtInit();
    ingestN(0x10, CLS_PHONE, SRC_BLE, 1, 1000);
    // Advance beyond the window: device must drop out.
    dtProcess(1000 + 5000UL + 1, 5000, FILTER_ALL);
    CHECK(dtSize() == 0);
    CHECK(dtPaxTotal() == 0);
}
TEST(dt_dedupe_within_cycle) {
    dtInit();
    char s[MAC_STR_CAP]; macToStr(s, sizeof(s), 0x20);
    dtIngest(0x20, s, CLS_PHONE, -40, SRC_BLE, true,  1000);
    dtIngest(0x20, s, CLS_PHONE, -40, SRC_BLE, false, 1000);  // same cycle, no ts
    dtProcess(1000, WINDOW_DURATION_MS, FILTER_ALL);
    size_t n = 0;
    const DeviceView* top = dtTopCommon(&n);
    CHECK(n == 1);
    CHECK(top[0].count == 1);   // only one timestamp recorded
}
TEST(dt_top_common_ordering) {
    dtInit();
    ingestN(0xA, CLS_PHONE, SRC_BLE, 1, 1000);
    ingestN(0xB, CLS_PHONE, SRC_BLE, 3, 1000);
    ingestN(0xC, CLS_PHONE, SRC_BLE, 2, 1000);
    dtProcess(1000, WINDOW_DURATION_MS, FILTER_ALL);
    size_t n = 0;
    const DeviceView* top = dtTopCommon(&n);
    CHECK(n == 3);
    CHECK(top[0].mac_key == 0xB);   // 3 detections
    CHECK(top[1].mac_key == 0xC);   // 2
    CHECK(top[2].mac_key == 0xA);   // 1
}
TEST(dt_recent_ordering) {
    dtInit();
    ingestN(0xA, CLS_PHONE, SRC_BLE, 1, 100);
    ingestN(0xB, CLS_PHONE, SRC_BLE, 1, 300);   // newest
    ingestN(0xC, CLS_PHONE, SRC_BLE, 1, 200);
    dtProcess(400, WINDOW_DURATION_MS, FILTER_ALL);
    size_t n = 0;
    const DeviceView* rec = dtRecent(&n);
    CHECK(n == 3);
    CHECK(rec[0].mac_key == 0xB);   // smallest age
    CHECK(rec[1].mac_key == 0xC);
    CHECK(rec[2].mac_key == 0xA);
}
TEST(dt_filter) {
    dtInit();
    ingestN(0xA, CLS_PHONE, SRC_BLE, 1, 1000);
    ingestN(0xB, CLS_WATCH, SRC_BLE, 1, 1000);
    dtProcess(1000, WINDOW_DURATION_MS, CLS_WATCH);
    size_t n = 0;
    const DeviceView* top = dtTopCommon(&n);
    CHECK(n == 1);
    CHECK(top[0].cls == CLS_WATCH);
}
TEST(dt_full_table_drops_and_counts) {
    dtInit();
    for (uint64_t i = 1; i <= DEV_MAX + 5; ++i) {
        ingestN(i, CLS_PHONE, SRC_BLE, 1, 1000);
    }
    dtProcess(1000, WINDOW_DURATION_MS, FILTER_ALL);
    CHECK(dtSize() == DEV_MAX);
    CHECK(dtDroppedInserts() == 5);
}
TEST(dt_count_saturates_at_ring) {
    dtInit();
    ingestN(0x30, CLS_PHONE, SRC_BLE, (int)DEV_TS_RING + 20, 1000);
    dtProcess(1000, WINDOW_DURATION_MS, FILTER_ALL);
    size_t n = 0;
    const DeviceView* top = dtTopCommon(&n);
    CHECK(n == 1);
    CHECK(top[0].count == DEV_TS_RING);   // saturates, never overflows
}

// ── Known cache ───────────────────────────────────────────────────────────────
TEST(kc_update_lookup) {
    kcInit();
    CHECK(kcUpdate(0x111, CLS_PHONE) == true);
    CHECK(kcUpdate(0x111, CLS_PHONE) == false);  // unchanged
    bool found = false;
    CHECK(kcLookup(0x111, &found) == CLS_PHONE);
    CHECK(found);
    kcLookup(0x999, &found);
    CHECK(!found);
}
TEST(kc_dirty_flag) {
    kcInit();
    CHECK(!kcDirty());
    kcUpdate(0x222, CLS_WATCH);
    CHECK(kcDirty());
    kcClearDirty();
    CHECK(!kcDirty());
}
TEST(kc_lru_eviction) {
    kcInit();
    // Fill the cache.
    for (uint64_t i = 1; i <= KNOWN_MAX; ++i) kcUpdate(i, CLS_PHONE);
    CHECK(kcSize() == KNOWN_MAX);
    // Touch mac 1 so it is most-recently-used; mac 2 becomes the LRU victim.
    bool f = false;
    kcLookup(1, &f);
    kcUpdate(0xF00D, CLS_WATCH);            // forces one eviction
    CHECK(kcSize() == KNOWN_MAX);
    kcLookup(1, &f);
    CHECK(f);                               // mac 1 survived
    kcLookup(2, &f);
    CHECK(!f);                              // mac 2 was evicted
}
TEST(kc_export_import_roundtrip) {
    kcInit();
    kcUpdate(0xAAA, CLS_PHONE);
    kcUpdate(0xBBB, CLS_WIFI);
    uint64_t macs[KNOWN_MAX];
    uint8_t  cls[KNOWN_MAX];
    size_t n = kcExport(macs, cls, KNOWN_MAX);
    CHECK(n == 2);
    kcInit();
    for (size_t i = 0; i < n; ++i) kcImport(macs[i], cls[i]);
    bool f = false;
    CHECK(kcLookup(0xAAA, &f) == CLS_PHONE);
    CHECK(kcLookup(0xBBB, &f) == CLS_WIFI);
}

// ── Persistence encode/decode ─────────────────────────────────────────────────
TEST(persist_roundtrip) {
    uint64_t macs[3]    = {0x1, 0x2, 0x3};
    uint8_t  cls[3]     = {CLS_PHONE, CLS_WATCH, CLS_WIFI};
    uint8_t  buf[64];
    size_t   len = persistEncode(buf, sizeof(buf), macs, cls, 3);
    CHECK(len == 7 + 3 * 9);

    uint64_t om[8]; uint8_t oc[8]; size_t on = 0;
    CHECK(persistDecode(buf, len, om, oc, 8, &on));
    CHECK(on == 3);
    CHECK(om[0] == 0x1 && oc[0] == CLS_PHONE);
    CHECK(om[2] == 0x3 && oc[2] == CLS_WIFI);
}
TEST(persist_bad_magic) {
    uint8_t buf[16] = {0};
    uint64_t om[4]; uint8_t oc[4]; size_t on = 0;
    CHECK(!persistDecode(buf, sizeof(buf), om, oc, 4, &on));
}
TEST(persist_truncated) {
    uint64_t macs[2] = {0x1, 0x2};
    uint8_t  cls[2]  = {CLS_PHONE, CLS_WATCH};
    uint8_t  buf[64];
    size_t   len = persistEncode(buf, sizeof(buf), macs, cls, 2);
    uint64_t om[4]; uint8_t oc[4]; size_t on = 0;
    CHECK(!persistDecode(buf, len - 3, om, oc, 4, &on));  // chop the last record
}
TEST(persist_skips_bad_record) {
    uint64_t macs[2] = {0x1, 0x0};          // second mac is invalid (0)
    uint8_t  cls[2]  = {CLS_PHONE, CLS_WATCH};
    uint8_t  buf[64];
    size_t   len = persistEncode(buf, sizeof(buf), macs, cls, 2);
    uint64_t om[4]; uint8_t oc[4]; size_t on = 0;
    CHECK(persistDecode(buf, len, om, oc, 4, &on));
    CHECK(on == 1);                          // bad record skipped
    CHECK(om[0] == 0x1);
}

// ── Entry point ──────────────────────────────────────────────────────────────
int main() {
    printf("\n=== PAX Counter Native Tests ===\n\n");
    printf("%d / %d checks passed\n", g_passed, g_tests);
    return (g_passed == g_tests) ? 0 : 1;
}
