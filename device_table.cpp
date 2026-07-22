// ─────────────────────────────────────────────────────────────────────────────
// Device Table — fixed-capacity live device store
// ─────────────────────────────────────────────────────────────────────────────
#include "device_table.h"
#include "classification.h"
#include "c_assert.h"

// ── Internal record (never exposed; the UI sees DeviceView copies) ───────────
struct DeviceRecord {
    bool     in_use;
    uint64_t mac_key;
    char     mac_str[MAC_STR_CAP];
    uint8_t  cls;
    int16_t  rssi;
    uint8_t  source;
    uint32_t ts[DEV_TS_RING];      // detection timestamps, pushed in time order
    uint8_t  ts_head;              // next write position, in [0, DEV_TS_RING)
    uint8_t  ts_count;             // valid entries, in [0, DEV_TS_RING]
    uint16_t count_in_window;      // == ts_count after prune (saturates at ring)
};

// ── Static storage (Rule 3) ──────────────────────────────────────────────────
static DeviceRecord g_dev[DEV_MAX];
static uint32_t     g_dropped = 0;

static int    g_pax_total = 0;
static int    g_pax_ble   = 0;
static int    g_pax_wifi  = 0;
static size_t g_size      = 0;

static DeviceView g_top[TOP_N_COMMON];
static size_t     g_top_n = 0;
static DeviceView g_recent[RECENT_N];
static size_t     g_recent_n = 0;

// ── Small helpers ────────────────────────────────────────────────────────────
static bool weakClass(uint8_t cls) {
    return cls == CLS_UNKNOWN || cls == CLS_OTHER_BLE;
}

// Bounded string copy; always NUL-terminates. cap must be > 0.
static void copyStr(char* dst, size_t cap, const char* src) {
    if (!c_assert(dst != nullptr) || !c_assert(cap > 0)) {
        return;
    }
    if (src == nullptr) {
        dst[0] = '\0';
        return;
    }
    size_t i = 0;
    for (; i < cap - 1 && src[i] != '\0'; ++i) {   // bound: cap (Rule 2)
        dst[i] = src[i];
    }
    dst[i] = '\0';
}

static int findIndex(uint64_t mac_key) {
    for (size_t i = 0; i < DEV_MAX; ++i) {          // bound: DEV_MAX (Rule 2)
        if (g_dev[i].in_use && g_dev[i].mac_key == mac_key) {
            return (int)i;
        }
    }
    return -1;
}

static int allocIndex() {
    for (size_t i = 0; i < DEV_MAX; ++i) {          // bound: DEV_MAX (Rule 2)
        if (!g_dev[i].in_use) {
            return (int)i;
        }
    }
    return -1;                                       // table full
}

// ── Timestamp ring ───────────────────────────────────────────────────────────
static void tsPush(DeviceRecord* rec, uint32_t now_ms) {
    if (!c_assert(rec != nullptr)) {
        return;
    }
    if (!c_assert(rec->ts_head < DEV_TS_RING)) {
        rec->ts_head = 0;                            // recover invariant
    }
    rec->ts[rec->ts_head] = now_ms;
    rec->ts_head = (uint8_t)((rec->ts_head + 1) % DEV_TS_RING);
    if (rec->ts_count < DEV_TS_RING) {
        rec->ts_count++;
    }
    rec->count_in_window = rec->ts_count;
}

static uint32_t tsNewest(const DeviceRecord* rec) {
    if (!c_assert(rec != nullptr) || !c_assert(rec->ts_count > 0)) {
        return 0;
    }
    size_t idx = (size_t)(rec->ts_head + DEV_TS_RING - 1) % DEV_TS_RING;
    return rec->ts[idx];
}

// Drop oldest detections older than window_ms. Timestamps are monotonic, so once
// the oldest is young enough we can stop. Guarded to DEV_TS_RING iters (Rule 2).
static void tsPrune(DeviceRecord* rec, uint32_t now_ms, uint32_t window_ms) {
    if (!c_assert(rec != nullptr)) {
        return;
    }
    size_t guard = 0;
    while (rec->ts_count > 0 && c_assert(guard < DEV_TS_RING)) {
        size_t oldest = (size_t)(rec->ts_head + DEV_TS_RING - rec->ts_count)
                        % DEV_TS_RING;
        uint32_t age = now_ms - rec->ts[oldest];     // wrap-safe unsigned age
        if (age <= window_ms) {
            break;
        }
        rec->ts_count--;
        guard++;
    }
    rec->count_in_window = rec->ts_count;
}

// ── View construction + top-K selection ──────────────────────────────────────
static void buildView(const DeviceRecord* rec, uint32_t now_ms, DeviceView* out) {
    if (!c_assert(rec != nullptr) || !c_assert(out != nullptr)) {
        return;
    }
    out->mac_key = rec->mac_key;
    copyStr(out->mac_str, MAC_STR_CAP, rec->mac_str);
    out->cls    = rec->cls;
    out->rssi   = rec->rssi;
    out->source = rec->source;
    out->count  = rec->count_in_window;
    uint32_t last = (rec->ts_count > 0) ? tsNewest(rec) : now_ms;
    out->last_seen_ms = last;
    out->age_ms       = now_ms - last;               // wrap-safe
}

// "Is a better ranked than b?" — by_recent picks recency (smaller age), else
// commonness (higher count); both tie-break on mac_key. No function pointer
// (Rule 9): the choice is a bool flag.
static bool viewBetter(const DeviceView* a, const DeviceView* b, bool by_recent) {
    if (!c_assert(a != nullptr) || !c_assert(b != nullptr)) {
        return false;
    }
    if (by_recent) {
        if (a->age_ms != b->age_ms) {
            return a->age_ms < b->age_ms;
        }
        return a->mac_key < b->mac_key;
    }
    if (a->count != b->count) {
        return a->count > b->count;
    }
    return a->mac_key < b->mac_key;
}

// Insert v into a descending sorted array capped at `cap`. Shifts bounded by cap.
static void insertView(DeviceView* arr, size_t* n, size_t cap,
                       const DeviceView* v, bool by_recent) {
    if (!c_assert(arr != nullptr) || !c_assert(n != nullptr)) {
        return;
    }
    if (!c_assert(v != nullptr) || !c_assert(cap > 0)) {
        return;
    }
    size_t pos = *n;
    for (size_t i = 0; i < *n; ++i) {                // bound: cap (Rule 2)
        if (viewBetter(v, &arr[i], by_recent)) {
            pos = i;
            break;
        }
    }
    if (pos >= cap) {
        return;                                       // not in the top-K
    }
    size_t end = (*n < cap) ? *n : cap - 1;
    for (size_t i = end; i > pos; --i) {             // bound: cap (Rule 2)
        arr[i] = arr[i - 1];
    }
    arr[pos] = *v;
    if (*n < cap) {
        (*n)++;
    }
}

static void buildSelections(uint32_t now_ms, uint8_t filter) {
    g_top_n    = 0;
    g_recent_n = 0;
    for (size_t i = 0; i < DEV_MAX; ++i) {           // bound: DEV_MAX (Rule 2)
        DeviceRecord* rec = &g_dev[i];
        if (!rec->in_use || rec->ts_count == 0) {
            continue;
        }
        if (filter != FILTER_ALL && rec->cls != filter) {
            continue;
        }
        DeviceView v;
        buildView(rec, now_ms, &v);
        insertView(g_top,    &g_top_n,    TOP_N_COMMON, &v, false);
        insertView(g_recent, &g_recent_n, RECENT_N,     &v, true);
    }
    (void)c_assert(g_top_n <= TOP_N_COMMON);
    (void)c_assert(g_recent_n <= RECENT_N);
}

static void initRecord(DeviceRecord* rec, uint64_t mac_key, const char* mac_str,
                       uint8_t cls, uint8_t source) {
    if (!c_assert(rec != nullptr) || !c_assert(mac_str != nullptr)) {
        return;
    }
    rec->in_use          = true;
    rec->mac_key         = mac_key;
    copyStr(rec->mac_str, MAC_STR_CAP, mac_str);
    rec->cls             = cls;
    rec->rssi            = 0;
    rec->source          = source;
    rec->ts_head         = 0;
    rec->ts_count        = 0;
    rec->count_in_window = 0;
}

// ── Public API ───────────────────────────────────────────────────────────────
void dtInit() {
    for (size_t i = 0; i < DEV_MAX; ++i) {           // bound: DEV_MAX (Rule 2)
        g_dev[i].in_use = false;
    }
    g_pax_total = g_pax_ble = g_pax_wifi = 0;
    g_size = 0;
    g_dropped = 0;
    g_top_n = g_recent_n = 0;
}

bool dtIngest(uint64_t mac_key, const char* mac_str, uint8_t cls,
              int rssi, uint8_t source, bool add_timestamp, uint32_t now_ms) {
    if (!c_assert(mac_key != 0) || !c_assert(mac_str != nullptr)) {
        return false;
    }
    if (!c_assert(classIsValid(cls))) {
        cls = CLS_UNKNOWN;
    }
    if (source != SRC_BLE && source != SRC_WIFI) {
        source = SRC_BLE;
    }
    int idx = findIndex(mac_key);
    if (idx < 0) {
        idx = allocIndex();
        if (idx < 0) {
            g_dropped++;                              // full: drop new device
            return false;
        }
        initRecord(&g_dev[idx], mac_key, mac_str, cls, source);
    }
    DeviceRecord* rec = &g_dev[idx];
    if (!c_assert(rec->in_use)) {
        return false;
    }
    if (!weakClass(cls) && rec->cls != cls) {
        rec->cls = cls;                               // upgrade to specific class
    }
    rec->rssi   = (int16_t)rssi;
    rec->source = source;
    if (add_timestamp) {
        tsPush(rec, now_ms);
    }
    return true;
}

void dtProcess(uint32_t now_ms, uint32_t window_ms, uint8_t filter) {
    if (!c_assert(filter == FILTER_ALL || classIsValid(filter))) {
        filter = FILTER_ALL;
    }
    int total = 0, ble = 0, wifi = 0;
    size_t size = 0;
    for (size_t i = 0; i < DEV_MAX; ++i) {           // bound: DEV_MAX (Rule 2)
        DeviceRecord* rec = &g_dev[i];
        if (!rec->in_use) {
            continue;
        }
        tsPrune(rec, now_ms, window_ms);
        if (rec->ts_count == 0) {
            rec->in_use = false;                      // no detections left in window
            continue;
        }
        size++;
        if (classIsPax(rec->cls)) {
            total++;
            if (rec->source == SRC_BLE) {
                ble++;
            } else {
                wifi++;
            }
        }
    }
    (void)c_assert(total <= (int)size);               // PAX is a subset of tracked
    g_pax_total = total;
    g_pax_ble   = ble;
    g_pax_wifi  = wifi;
    g_size      = size;
    buildSelections(now_ms, filter);
}

int    dtPaxTotal()       { return g_pax_total; }
int    dtPaxBle()         { return g_pax_ble; }
int    dtPaxWifi()        { return g_pax_wifi; }
size_t dtSize()           { return g_size; }
uint32_t dtDroppedInserts() { return g_dropped; }

const DeviceView* dtTopCommon(size_t* count) {
    if (c_assert(count != nullptr)) {
        *count = g_top_n;
    }
    return g_top;
}

const DeviceView* dtRecent(size_t* count) {
    if (c_assert(count != nullptr)) {
        *count = g_recent_n;
    }
    return g_recent;
}
