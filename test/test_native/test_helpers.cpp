// ─────────────────────────────────────────────────────────────────────────────
// PAX Counter M5Stack - Native Unit Tests
// ─────────────────────────────────────────────────────────────────────────────
// Run with:  pio test -e native
//
// These tests exercise pure-C++ code that has no Arduino / ESP-IDF dependency:
//   - truncateString()
//   - macToString()
//   - PaxHistory

// Minimal shim for snprintf (already in <cstdio> on the host)
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <string>
#include <cassert>

// ── Inline the testable code so we don't need the full build ─────────────────
// (PlatformIO native test env can't pull in ESP-IDF headers.)

#include "../../include/types.h"
#include "../../include/history.h"

// ── Test framework (minimal, no external deps) ──────────────────────────────
static int g_tests  = 0;
static int g_passed = 0;

#define TEST(name)                                         \
    static void test_##name();                             \
    static struct Register_##name {                        \
        Register_##name() { test_##name(); }               \
    } reg_##name;                                          \
    static void test_##name()

#define EXPECT_EQ(a, b)                                    \
    do {                                                   \
        g_tests++;                                         \
        if ((a) == (b)) { g_passed++; }                    \
        else {                                             \
            printf("  FAIL: %s:%d  expected '%s' == '%s'\n", \
                   __FILE__, __LINE__, #a, #b);            \
        }                                                  \
    } while (0)

#define EXPECT_TRUE(x)                                     \
    do {                                                   \
        g_tests++;                                         \
        if ((x)) { g_passed++; }                           \
        else {                                             \
            printf("  FAIL: %s:%d  expected true: %s\n",   \
                   __FILE__, __LINE__, #x);                \
        }                                                  \
    } while (0)

// ── truncateString tests ─────────────────────────────────────────────────────

TEST(truncate_shorter_unchanged) {
    EXPECT_EQ(truncateString("abc", 5), std::string("abc"));
}

TEST(truncate_exact_fit) {
    EXPECT_EQ(truncateString("abcde", 5), std::string("abcde"));
}

TEST(truncate_longer_gets_dot) {
    EXPECT_EQ(truncateString("abcdef", 5), std::string("abcd."));
}

TEST(truncate_width_1) {
    EXPECT_EQ(truncateString("hello", 1), std::string("h"));
}

TEST(truncate_width_0) {
    EXPECT_EQ(truncateString("hello", 0), std::string(""));
}

TEST(truncate_empty) {
    EXPECT_EQ(truncateString("", 5), std::string(""));
}

// ── macToString tests ────────────────────────────────────────────────────────

TEST(mac_to_string_basic) {
    uint64_t mac = 0xAABBCCDDEEFF;
    EXPECT_EQ(macToString(mac), std::string("aa:bb:cc:dd:ee:ff"));
}

TEST(mac_to_string_zero) {
    EXPECT_EQ(macToString(0), std::string("00:00:00:00:00:00"));
}

// ── PaxHistory tests ─────────────────────────────────────────────────────────

TEST(history_empty) {
    PaxHistory h;
    EXPECT_EQ(h.count, 0);
    EXPECT_EQ(h.newest(), 0);
    EXPECT_EQ(h.windowMax(), 0);
    EXPECT_EQ(h.average(), 0L);
}

TEST(history_push_and_read) {
    PaxHistory h;
    h.push(10);
    h.push(20);
    h.push(30);
    EXPECT_EQ(h.count, 3);
    EXPECT_EQ(h.at(0), 10);   // oldest
    EXPECT_EQ(h.at(2), 30);   // newest
    EXPECT_EQ(h.newest(), 30);
}

TEST(history_peak) {
    PaxHistory h;
    h.push(5);
    h.push(42);
    h.push(3);
    EXPECT_EQ(h.peak, 42);
}

TEST(history_window_max) {
    PaxHistory h;
    for (int i = 0; i < 10; i++) h.push(i);
    EXPECT_EQ(h.windowMax(), 9);
}

TEST(history_average) {
    PaxHistory h;
    h.push(10);
    h.push(20);
    h.push(30);
    EXPECT_EQ(h.average(), 20L);
}

TEST(history_wrap_around) {
    PaxHistory h;
    for (int i = 0; i < PaxHistory::CAPACITY + 10; i++)
        h.push(i);
    EXPECT_EQ(h.count, PaxHistory::CAPACITY);
    // Oldest should be 10 (first 10 were evicted)
    EXPECT_EQ(h.at(0), 10);
    EXPECT_EQ(h.newest(), PaxHistory::CAPACITY + 9);
}

// ── PAX_RELEVANT set tests ──────────────────────────────────────────────────

TEST(pax_relevant_includes_phone) {
    EXPECT_TRUE(PAX_RELEVANT.count("Phone") > 0);
}

TEST(pax_relevant_includes_wifi) {
    EXPECT_TRUE(PAX_RELEVANT.count("WiFi") > 0);
}

TEST(pax_relevant_excludes_beacons) {
    EXPECT_TRUE(PAX_RELEVANT.count("iBeacon") == 0);
    EXPECT_TRUE(PAX_RELEVANT.count("Eddystone") == 0);
}

TEST(pax_relevant_excludes_sensors) {
    EXPECT_TRUE(PAX_RELEVANT.count("Sensor") == 0);
}

// ── Entry point ──────────────────────────────────────────────────────────────
int main() {
    printf("\n=== PAX Counter Native Tests ===\n\n");
    // Tests already ran via static constructors
    printf("\n%d / %d tests passed\n", g_passed, g_tests);
    return (g_passed == g_tests) ? 0 : 1;
}
