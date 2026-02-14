// ─────────────────────────────────────────────────────────────────────────────
// PAX Counter M5Stack - Display Manager
// ─────────────────────────────────────────────────────────────────────────────
#include "display_manager.h"
#include "config.h"
#include "types.h"
#include "pax_store.h"

#include <M5Unified.h>
#include <vector>
#include <string>
#include <algorithm>

// ── State ────────────────────────────────────────────────────────────────────
static DisplayPage g_page = DisplayPage::DASHBOARD;

static std::vector<std::string> g_filters;
static int         g_filterIdx    = 0;
static std::string g_filterLabel  = "ALL";
static bool        g_forceRedraw  = true;

// Change-detection caches
static int         g_lastPax      = -1;
static int         g_lastBle      = -1;
static int         g_lastWifi     = -1;
static std::string g_lastTopSum;
static std::string g_lastRecSum;
static DisplayPage g_lastPage     = DisplayPage::DASHBOARD;

// Touch debounce
static bool g_touchA_prev = false;
static bool g_touchB_prev = false;
static bool g_touchC_prev = false;

// ── Helpers ──────────────────────────────────────────────────────────────────

// Build a compact summary of a device list for change detection.
static std::string listSummary(const std::vector<MacActivityInfo>& v,
                               bool includeCount) {
    std::string s;
    s.reserve(v.size() * 40);
    for (const auto& d : v) {
        s += std::to_string(d.mac_key);
        s += ';';
        s += d.classification;
        s += ';';
        if (includeCount) {
            s += std::to_string(d.count_in_window);
        } else if (!d.timestamps.empty()) {
            s += std::to_string(d.timestamps.back());
        }
        s += '|';
    }
    return s;
}

// Map a value to a green-yellow-red gradient (low=green, high=red).
static uint16_t heatColor(int val, int maxVal) {
    if (maxVal <= 0) return TFT_GREEN;
    float t = std::min((float)val / maxVal, 1.0f);
    uint8_t r, g;
    if (t < 0.5f) {
        r = (uint8_t)(255 * t * 2);
        g = 255;
    } else {
        r = 255;
        g = (uint8_t)(255 * (1.0f - t) * 2);
    }
    // Convert to RGB565
    return ((r >> 3) << 11) | ((g >> 2) << 5) | (0 >> 3);
}

// ── Page: Dashboard ──────────────────────────────────────────────────────────
static void drawDashboard(PaxHistory& hist) {
    int pax  = getPaxTotal();
    int ble  = getPaxBle();
    int wifi = getPaxWifi();

    bool paxChanged = (pax != g_lastPax || ble != g_lastBle || wifi != g_lastWifi);

    if (paxChanged || g_forceRedraw) {
        // ── Top region: big PAX count + BLE/WiFi breakdown ───────────────
        M5.Lcd.fillRect(0, 0, SCREEN_W, PAX_REGION_H, TFT_BLACK);
        M5.Lcd.setTextSize(PAX_FONT_SZ);
        M5.Lcd.setTextColor(TFT_WHITE, TFT_BLACK);
        M5.Lcd.setCursor(DETAIL_X, PAX_PAD / 2);
        M5.Lcd.printf("%d", pax);

        // Small BLE / WiFi breakdown to the right
        M5.Lcd.setTextSize(1);
        M5.Lcd.setTextColor(TFT_CYAN, TFT_BLACK);
        M5.Lcd.setCursor(SCREEN_W - 110, 4);
        M5.Lcd.printf("BLE:%d WiFi:%d", ble, wifi);
        M5.Lcd.setTextColor(TFT_WHITE, TFT_BLACK);

        g_lastPax  = pax;
        g_lastBle  = ble;
        g_lastWifi = wifi;
    }

    // ── Always redraw the graph (it changes every sample + animation) ────
    if (g_forceRedraw || paxChanged) {
        M5.Lcd.fillRect(GRAPH_X - 2, GRAPH_Y - 2,
                         GRAPH_W + 4, GRAPH_H + 14, TFT_BLACK);

        int maxVal = std::max(hist.windowMax(), 1);
        int barStep = GRAPH_BAR_W + GRAPH_BAR_GAP;
        int maxBars = GRAPH_W / barStep;
        int start   = (hist.count > maxBars) ? hist.count - maxBars : 0;
        int nBars   = hist.count - start;

        // Draw bars right-aligned
        int xOff = GRAPH_X + GRAPH_W - (nBars * barStep);
        for (int i = start; i < hist.count; i++) {
            int v = hist.at(i);
            int barH = (v * GRAPH_H) / maxVal;
            if (barH < 1 && v > 0) barH = 1;
            int x = xOff + (i - start) * barStep;
            int y = GRAPH_Y + GRAPH_H - barH;

            uint16_t col = heatColor(v, maxVal);
            M5.Lcd.fillRect(x, y, GRAPH_BAR_W, barH, col);
        }

        // Axis labels
        M5.Lcd.setTextSize(1);
        M5.Lcd.setTextColor(TFT_DARKGREY, TFT_BLACK);
        int labelY = GRAPH_Y + GRAPH_H + 2;
        M5.Lcd.setCursor(GRAPH_X, labelY);

        int minsAgo = hist.count;
        M5.Lcd.printf("-%dm", minsAgo);
        M5.Lcd.setCursor(GRAPH_X + GRAPH_W - 18, labelY);
        M5.Lcd.print("now");
    }

    // ── Stats line below graph ───────────────────────────────────────────
    if (paxChanged || g_forceRedraw) {
        int statsY = GRAPH_Y + GRAPH_H + 14;
        M5.Lcd.fillRect(0, statsY, SCREEN_W, 24, TFT_BLACK);
        M5.Lcd.setTextSize(1);
        M5.Lcd.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
        M5.Lcd.setCursor(DETAIL_X, statsY);
        M5.Lcd.printf("Peak:%d  Avg:%ld  DB:%u",
                       hist.peak, hist.average(), (unsigned)getDbSize());

        // Uptime
        unsigned long sec = millis() / 1000;
        unsigned long h = sec / 3600;
        unsigned long m = (sec % 3600) / 60;
        M5.Lcd.setCursor(DETAIL_X, statsY + 10);
        M5.Lcd.printf("Up: %luh%02lum", h, m);
    }

    // ── Bottom label ─────────────────────────────────────────────────────
    if (g_forceRedraw) {
        int bottomY = SCREEN_H - 12;
        M5.Lcd.fillRect(0, bottomY, SCREEN_W, 12, TFT_BLACK);
        M5.Lcd.setTextSize(1);
        M5.Lcd.setTextColor(TFT_DARKGREY, TFT_BLACK);

        M5.Lcd.setCursor(30, bottomY);
        M5.Lcd.print("<");
        M5.Lcd.setCursor(130, bottomY);
        M5.Lcd.print("[Dashboard]");
        M5.Lcd.setCursor(280, bottomY);
        M5.Lcd.print(">");
    }
}

// ── Page: Device detail list ─────────────────────────────────────────────────
static void drawDetailPage() {
    int pax = getPaxTotal();
    const auto& topCommon = getTopCommon();
    const auto& recent    = getRecentList();

    // Change detection
    std::string topSum = listSummary(topCommon, true);
    std::string recSum = listSummary(recent, false);
    bool paxChanged  = (pax != g_lastPax);
    bool dataChanged = (topSum != g_lastTopSum || recSum != g_lastRecSum);

    // ── PAX count header ─────────────────────────────────────────────────
    if (paxChanged || g_forceRedraw) {
        M5.Lcd.fillRect(0, 0, SCREEN_W, PAX_REGION_H, TFT_BLACK);
        M5.Lcd.setTextSize(PAX_FONT_SZ);
        M5.Lcd.setTextColor(TFT_WHITE, TFT_BLACK);
        M5.Lcd.setCursor(DETAIL_X, PAX_PAD / 2);
        M5.Lcd.printf("%d", pax);
        g_lastPax = pax;
    }

    if (!dataChanged && !g_forceRedraw) return;
    g_lastTopSum = topSum;
    g_lastRecSum = recSum;

    // ── Detail region ────────────────────────────────────────────────────
    M5.Lcd.fillRect(0, DETAIL_Y, SCREEN_W, DETAIL_H, TFT_BLACK);
    M5.Lcd.setTextSize(DETAIL_FONT_SZ);
    int y = DETAIL_Y;

    // Filter label
    M5.Lcd.setTextColor(TFT_YELLOW, TFT_BLACK);
    M5.Lcd.setCursor(DETAIL_X, y);
    M5.Lcd.printf("Filter: %s", g_filterLabel.c_str());
    y += DETAIL_FONT_H + 3;
    M5.Lcd.setTextColor(TFT_WHITE, TFT_BLACK);

    // Top common
    M5.Lcd.setCursor(DETAIL_X, y);
    M5.Lcd.print("Most Common:");
    y += DETAIL_FONT_H + 1;

    for (size_t i = 0; i < topCommon.size(); i++) {
        const auto& d = topCommon[i];
        if (y + DETAIL_FONT_H > DETAIL_Y + DETAIL_H) break;

        M5.Lcd.setCursor(DETAIL_X + LIST_INDENT, y);
        std::string tl = truncateString(d.classification, CLS_DISPLAY_W - 4);
        char src = (d.source == ScanSource::WIFI) ? 'W' : 'B';
        M5.Lcd.printf("%zu.%s %-5s (%u,%ddBm)%c",
                       i + 1, d.mac_str.c_str(), tl.c_str(),
                       (unsigned)d.count_in_window, d.rssi, src);
        y += DETAIL_FONT_H + DETAIL_SPACING;
    }
    y += 3;

    // Separator
    if (y < DETAIL_Y + DETAIL_H - DETAIL_FONT_H) {
        M5.Lcd.drawFastHLine(DETAIL_X, y, SCREEN_W - DETAIL_X * 2, TFT_DARKGREY);
        y += 4;
    }

    // Recent sightings
    if (y < DETAIL_Y + DETAIL_H - DETAIL_FONT_H) {
        M5.Lcd.setCursor(DETAIL_X, y);
        M5.Lcd.print("Recent Sightings:");
        y += DETAIL_FONT_H + 1;
    }

    for (size_t i = 0; i < recent.size(); i++) {
        const auto& d = recent[i];
        if (y + DETAIL_FONT_H > DETAIL_Y + DETAIL_H) break;

        unsigned long ago = 0;
        if (!d.timestamps.empty())
            ago = (millis() - d.timestamps.back()) / 1000;

        M5.Lcd.setCursor(DETAIL_X + LIST_INDENT, y);
        std::string tl = truncateString(d.classification, CLS_DISPLAY_W - 4);
        char src = (d.source == ScanSource::WIFI) ? 'W' : 'B';
        M5.Lcd.printf(" %s %-5s (%lus,%ddBm)%c",
                       d.mac_str.c_str(), tl.c_str(), ago, d.rssi, src);
        y += DETAIL_FONT_H + DETAIL_SPACING;
    }

    // Bottom labels
    {
        int bottomY = SCREEN_H - 12;
        M5.Lcd.fillRect(0, bottomY, SCREEN_W, 12, TFT_BLACK);
        M5.Lcd.setTextSize(1);
        M5.Lcd.setTextColor(TFT_DARKGREY, TFT_BLACK);
        M5.Lcd.setCursor(20, bottomY);
        M5.Lcd.print("< Filt");
        M5.Lcd.setCursor(133, bottomY);
        M5.Lcd.print("[Detail]");
        M5.Lcd.setCursor(264, bottomY);
        M5.Lcd.print("Filt >");
    }
}

// ── Public API ───────────────────────────────────────────────────────────────

void initDisplay() {
    M5.Lcd.setRotation(1);
    M5.Lcd.fillScreen(TFT_BLACK);
    M5.Lcd.setTextColor(TFT_WHITE, TFT_BLACK);

    // Build filter list
    g_filters.push_back("ALL");
    for (const auto& c : PAX_RELEVANT)
        g_filters.push_back(c);
    g_filterLabel = g_filters[0];
    g_forceRedraw = true;
}

void handleTouch() {
    bool curA = false, curB = false, curC = false;

    int tc = M5.Touch.getCount();
    if (tc > 0) {
        auto tp = M5.Touch.getDetail(0);
        if (tp.y >= TOUCH_Y_MIN && tp.y <= TOUCH_Y_MAX) {
            if (tp.x >= TOUCH_A_XMIN && tp.x <= TOUCH_A_XMAX) curA = true;
            if (tp.x >= TOUCH_B_XMIN && tp.x <= TOUCH_B_XMAX) curB = true;
            if (tp.x >= TOUCH_C_XMIN && tp.x <= TOUCH_C_XMAX) curC = true;
        }
    }

    // Rising-edge detection
    bool pressA = curA && !g_touchA_prev;
    bool pressB = curB && !g_touchB_prev;
    bool pressC = curC && !g_touchC_prev;
    g_touchA_prev = curA;
    g_touchB_prev = curB;
    g_touchC_prev = curC;

    // Middle button: switch page
    if (pressB) {
        int p = (int)g_page + 1;
        if (p >= (int)DisplayPage::PAGE_COUNT) p = 0;
        g_page = (DisplayPage)p;
        g_forceRedraw = true;
        Serial.printf("[Disp] Page -> %d\n", p);
    }

    // Left/Right: cycle filter (detail page only)
    if (g_page == DisplayPage::DETAIL) {
        bool changed = false;
        if (pressA && !g_filters.empty()) {
            g_filterIdx--;
            if (g_filterIdx < 0) g_filterIdx = (int)g_filters.size() - 1;
            changed = true;
        }
        if (pressC && !g_filters.empty()) {
            g_filterIdx++;
            if (g_filterIdx >= (int)g_filters.size()) g_filterIdx = 0;
            changed = true;
        }
        if (changed) {
            g_filterLabel = g_filters[g_filterIdx];
            g_forceRedraw = true;
            g_lastTopSum.clear();
            g_lastRecSum.clear();
            Serial.printf("[Disp] Filter -> %s\n", g_filterLabel.c_str());
        }
    }
}

void renderFrame(PaxHistory& history) {
    if (g_page != g_lastPage) {
        M5.Lcd.fillScreen(TFT_BLACK);
        g_forceRedraw = true;
        g_lastPage = g_page;
    }

    switch (g_page) {
        case DisplayPage::DASHBOARD: drawDashboard(history); break;
        case DisplayPage::DETAIL:    drawDetailPage();       break;
        default: break;
    }
    g_forceRedraw = false;
}

const std::string& currentFilterLabel() {
    return g_filterLabel;
}
