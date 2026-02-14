// ─────────────────────────────────────────────────────────────────────────────
// PAX Counter M5Stack - Device Database & Persistence
// ─────────────────────────────────────────────────────────────────────────────
#include "pax_store.h"
#include "config.h"
#include "types.h"

#include <SPIFFS.h>
#include <ArduinoJson.h>
#include <algorithm>
#include <map>
#include <deque>

// ── Internal state ───────────────────────────────────────────────────────────
static std::map<uint64_t, MacActivityInfo> g_db;
static std::map<uint64_t, std::string>     g_knownCls;     // Persistent cache
static std::deque<uint64_t>                g_clsLru;       // LRU order for cache
static bool          g_clsDirty      = false;
static unsigned long g_lastSaveMs    = 0;

static std::vector<MacActivityInfo> g_topCommon;
static std::vector<MacActivityInfo> g_recent;

static int g_paxTotal = 0;
static int g_paxBle   = 0;
static int g_paxWifi  = 0;

// ── LRU helper ───────────────────────────────────────────────────────────────
static void touchLru(uint64_t key) {
    // Remove existing entry (O(n) but n is bounded)
    g_clsLru.erase(std::remove(g_clsLru.begin(), g_clsLru.end(), key),
                    g_clsLru.end());
    g_clsLru.push_back(key);
    // Evict oldest if over cap
    while (g_clsLru.size() > MAX_KNOWN_CLASSIFICATIONS) {
        uint64_t oldest = g_clsLru.front();
        g_clsLru.pop_front();
        g_knownCls.erase(oldest);
    }
}

// ── Public: ingest a sighting ────────────────────────────────────────────────
void paxStoreIngest(uint64_t macKey, const std::string& macStr,
                    const std::string& label, int rssi,
                    ScanSource source, bool addTimestamp) {

    auto it = g_db.find(macKey);
    if (it == g_db.end()) {
        // New device this session
        MacActivityInfo info(macKey, macStr, source);
        std::string cls = label;

        // If classification is vague, check persistent cache
        if (cls == CLS_UNKNOWN || cls == CLS_OTHER_BLE) {
            auto kit = g_knownCls.find(macKey);
            if (kit != g_knownCls.end()) cls = kit->second;
        }
        info.classification = cls;
        info.rssi = rssi;

        auto [inserted, _] = g_db.emplace(macKey, std::move(info));
        it = inserted;
    } else {
        // Existing device — upgrade classification if we have better info
        if (label != CLS_UNKNOWN && label != CLS_OTHER_BLE &&
            it->second.classification != label) {
            it->second.classification = label;
        }
        it->second.rssi = rssi;
    }

    // Record timestamp (once per scan cycle per MAC)
    if (addTimestamp)
        it->second.timestamps.push_back(millis());

    // Update persistent classification cache
    const std::string& cls = it->second.classification;
    if (cls != CLS_UNKNOWN && cls != CLS_OTHER_BLE) {
        auto kit = g_knownCls.find(macKey);
        if (kit == g_knownCls.end() || kit->second != cls) {
            g_knownCls[macKey] = cls;
            touchLru(macKey);
            g_clsDirty = true;
        }
    }
}

// ── Public: window processing ────────────────────────────────────────────────
void processActivityData(const std::string& filterLabel) {
    unsigned long now = millis();
    std::vector<uint64_t> prune;
    int total = 0, ble = 0, wifi = 0;

    for (auto& [key, info] : g_db) {
        // Prune old timestamps
        while (!info.timestamps.empty() &&
               (now - info.timestamps.front() > WINDOW_DURATION_MS)) {
            info.timestamps.pop_front();
        }
        info.count_in_window = info.timestamps.size();

        if (info.count_in_window == 0) {
            prune.push_back(key);
        } else if (PAX_RELEVANT.count(info.classification)) {
            total++;
            if (info.source == ScanSource::BLE)  ble++;
            if (info.source == ScanSource::WIFI) wifi++;
        }
    }
    for (uint64_t k : prune) g_db.erase(k);

    g_paxTotal = total;
    g_paxBle   = ble;
    g_paxWifi  = wifi;

    // ── Build filtered, sorted lists for display ─────────────────────────────
    std::vector<MacActivityInfo> pool;
    pool.reserve(g_db.size());
    for (const auto& [key, info] : g_db) {
        if (info.count_in_window == 0) continue;
        if (filterLabel != "ALL" && info.classification != filterLabel) continue;
        pool.push_back(info);
    }

    // Top common (most detections)
    g_topCommon.clear();
    {
        size_t n = std::min(pool.size(), (size_t)TOP_N_COMMON);
        g_topCommon.resize(n);
        if (n > 0) {
            std::partial_sort_copy(pool.begin(), pool.end(),
                                   g_topCommon.begin(), g_topCommon.end());
        }
    }

    // Recently sighted
    g_recent.clear();
    {
        std::vector<MacActivityInfo> recPool;
        recPool.reserve(pool.size());
        for (auto& info : pool) {
            if (!info.timestamps.empty())
                recPool.push_back(info);
        }
        std::sort(recPool.begin(), recPool.end(), ByRecency());
        size_t n = std::min(recPool.size(), (size_t)RECENT_N);
        if (n > 0)
            g_recent.assign(recPool.begin(), recPool.begin() + n);
    }
}

// ── Persistence ──────────────────────────────────────────────────────────────
void loadClassifications() {
    if (!SPIFFS.begin(true)) {
        Serial.println("[Store] SPIFFS mount failed");
        return;
    }
    File f = SPIFFS.open(PERSISTENCE_FILE, FILE_READ);
    if (!f || f.size() == 0) {
        if (f) f.close();
        Serial.println("[Store] No saved classifications");
        return;
    }
    JsonDocument doc;
    auto err = deserializeJson(doc, f);
    f.close();
    if (err) {
        Serial.printf("[Store] JSON parse error: %s\n", err.c_str());
        return;
    }
    for (JsonPair kv : doc.as<JsonObject>()) {
        uint64_t key = 0;
        if (sscanf(kv.key().c_str(), "%llu", &key) == 1) {
            g_knownCls[key] = kv.value().as<std::string>();
            g_clsLru.push_back(key);
        }
    }
    // Trim to cap
    while (g_clsLru.size() > MAX_KNOWN_CLASSIFICATIONS) {
        g_knownCls.erase(g_clsLru.front());
        g_clsLru.pop_front();
    }
    Serial.printf("[Store] Loaded %u classifications\n",
                  (unsigned)g_knownCls.size());
}

void saveClassificationsIfNeeded() {
    if (!g_clsDirty) return;
    unsigned long now = millis();
    if (now - g_lastSaveMs < SAVE_INTERVAL_MS) return;

    JsonDocument doc;
    for (const auto& [key, label] : g_knownCls) {
        char buf[24];
        snprintf(buf, sizeof(buf), "%llu", (unsigned long long)key);
        doc[buf] = label;
    }

    File f = SPIFFS.open(PERSISTENCE_FILE, FILE_WRITE);
    if (!f) {
        Serial.println("[Store] Failed to open file for writing");
        return;
    }
    if (serializeJson(doc, f) == 0) {
        Serial.println("[Store] JSON write failed");
    } else {
        Serial.printf("[Store] Saved %u classifications\n",
                      (unsigned)g_knownCls.size());
        g_clsDirty = false;
        g_lastSaveMs = now;
    }
    f.close();
}

// ── Accessors ────────────────────────────────────────────────────────────────
int  getPaxTotal()  { return g_paxTotal; }
int  getPaxBle()    { return g_paxBle; }
int  getPaxWifi()   { return g_paxWifi; }
const std::vector<MacActivityInfo>& getTopCommon()  { return g_topCommon; }
const std::vector<MacActivityInfo>& getRecentList() { return g_recent; }
size_t getDbSize()  { return g_db.size(); }
