// ─────────────────────────────────────────────────────────────────────────────
// Known-Classification Cache — fixed-size MAC -> class map with LRU eviction
// ─────────────────────────────────────────────────────────────────────────────
#include "known_cache.h"
#include "config.h"
#include "classification.h"
#include "c_assert.h"

struct KnownEntry {
    bool     in_use;
    uint64_t mac;
    uint8_t  cls;
    uint32_t last_used;      // monotonic recency stamp; larger = more recent
};

static KnownEntry g_known[KNOWN_MAX];
static uint32_t   g_clock = 0;      // bumped on every access to order the LRU
static bool       g_dirty = false;

// Next recency stamp. Saturates instead of wrapping so ordering never inverts
// (49 days of 5 s scans is ~840k bumps, far below UINT32_MAX; the clamp is belt
// and braces).
static uint32_t nextClock() {
    if (g_clock < 0xFFFFFFFFu) {
        g_clock++;
    }
    return g_clock;
}

static int findSlot(uint64_t mac) {
    for (size_t i = 0; i < KNOWN_MAX; ++i) {         // bound: KNOWN_MAX (Rule 2)
        if (g_known[i].in_use && g_known[i].mac == mac) {
            return (int)i;
        }
    }
    return -1;
}

// Free slot if any, else the least-recently-used slot to evict. Always returns a
// valid index in [0, KNOWN_MAX).
static int slotForInsert() {
    int lru = 0;
    uint32_t lru_used = 0xFFFFFFFFu;
    for (size_t i = 0; i < KNOWN_MAX; ++i) {         // bound: KNOWN_MAX (Rule 2)
        if (!g_known[i].in_use) {
            return (int)i;
        }
        if (g_known[i].last_used < lru_used) {
            lru_used = g_known[i].last_used;
            lru = (int)i;
        }
    }
    (void)c_assert(lru >= 0 && (size_t)lru < KNOWN_MAX);
    return lru;
}

void kcInit() {
    for (size_t i = 0; i < KNOWN_MAX; ++i) {         // bound: KNOWN_MAX (Rule 2)
        g_known[i].in_use = false;
    }
    g_clock = 0;
    g_dirty = false;
}

uint8_t kcLookup(uint64_t mac, bool* found) {
    if (!c_assert(mac != 0)) {
        if (found) {
            *found = false;
        }
        return CLS_UNKNOWN;
    }
    int idx = findSlot(mac);
    if (idx < 0) {
        if (found) {
            *found = false;
        }
        return CLS_UNKNOWN;
    }
    if (!c_assert((size_t)idx < KNOWN_MAX)) {
        if (found) {
            *found = false;
        }
        return CLS_UNKNOWN;
    }
    g_known[idx].last_used = nextClock();            // hit refreshes recency
    if (found) {
        *found = true;
    }
    return g_known[idx].cls;
}

bool kcUpdate(uint64_t mac, uint8_t cls) {
    if (!c_assert(mac != 0) || !c_assert(classIsValid(cls))) {
        return false;
    }
    int idx = findSlot(mac);
    if (idx >= 0) {
        if (g_known[idx].cls == cls) {
            g_known[idx].last_used = nextClock();
            return false;                            // already mapped this way
        }
        g_known[idx].cls = cls;
        g_known[idx].last_used = nextClock();
        g_dirty = true;
        return true;
    }
    idx = slotForInsert();
    if (!c_assert((size_t)idx < KNOWN_MAX)) {
        return false;
    }
    g_known[idx].in_use    = true;
    g_known[idx].mac       = mac;
    g_known[idx].cls       = cls;
    g_known[idx].last_used = nextClock();
    g_dirty = true;
    return true;
}

size_t kcSize() {
    size_t n = 0;
    for (size_t i = 0; i < KNOWN_MAX; ++i) {         // bound: KNOWN_MAX (Rule 2)
        if (g_known[i].in_use) {
            n++;
        }
    }
    return n;
}

size_t kcExport(uint64_t* macs, uint8_t* classes, size_t cap) {
    if (!c_assert(macs != nullptr) || !c_assert(classes != nullptr)) {
        return 0;
    }
    size_t n = 0;
    for (size_t i = 0; i < KNOWN_MAX && n < cap; ++i) {  // bound: KNOWN_MAX
        if (g_known[i].in_use) {
            macs[n]    = g_known[i].mac;
            classes[n] = g_known[i].cls;
            n++;
        }
    }
    return n;
}

bool kcImport(uint64_t mac, uint8_t cls) {
    if (!c_assert(mac != 0) || !c_assert(classIsValid(cls))) {
        return false;
    }
    int idx = findSlot(mac);
    if (idx < 0) {
        idx = slotForInsert();
    }
    if (!c_assert((size_t)idx < KNOWN_MAX)) {
        return false;
    }
    g_known[idx].in_use    = true;
    g_known[idx].mac       = mac;
    g_known[idx].cls       = cls;
    g_known[idx].last_used = nextClock();
    return true;
}

bool kcDirty()      { return g_dirty; }
void kcClearDirty() { g_dirty = false; }
