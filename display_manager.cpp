// ─────────────────────────────────────────────────────────────────────────────
// Display Manager — two-page touch UI
// ─────────────────────────────────────────────────────────────────────────────
#include "display_manager.h"
#include "config.h"
#include "types.h"
#include "classification.h"
#include "pax_store.h"
#include "device_table.h"
#include "c_assert.h"

#include <M5Unified.h>

// ── State (smallest scope: file-local, Rule 6) ───────────────────────────────
static DisplayPage g_page = DisplayPage::DASHBOARD;

static uint8_t g_filters[CLS_COUNT + 1];   // FILTER_ALL + each PAX class
static size_t  g_filter_n   = 0;
static size_t  g_filter_idx = 0;

static bool     g_force      = true;
static uint32_t g_dash_sig   = 0;          // change signature for the dashboard
static uint32_t g_top_sig    = 0;          // ...for the top-common list
static uint32_t g_recent_sig = 0;          // ...for the recent list
static int      g_last_pax   = -1;
static DisplayPage g_last_page = DisplayPage::DASHBOARD;

static bool g_touchA_prev = false;
static bool g_touchB_prev = false;
static bool g_touchC_prev = false;

// ── Small helpers ────────────────────────────────────────────────────────────
static uint32_t mix(uint32_t h, uint32_t x) {
    return (h ^ x) * 16777619u;             // FNV-style, no allocation
}

static uint32_t listSignature(const DeviceView* v, size_t n, bool use_age) {
    if (!c_assert(v != nullptr) || !c_assert(n <= (size_t)TOP_N_COMMON + RECENT_N)) {
        return 0;
    }
    uint32_t h = 2166136261u;
    for (size_t i = 0; i < n; ++i) {         // bound: list cap (Rule 2)
        h = mix(h, (uint32_t)v[i].mac_key);
        h = mix(h, (uint32_t)(v[i].mac_key >> 32));
        h = mix(h, v[i].cls);
        h = mix(h, use_age ? (v[i].age_ms / 1000) : v[i].count);
        h = mix(h, (uint32_t)(uint16_t)v[i].rssi);
    }
    return mix(h, (uint32_t)n);
}

// Green→yellow→red heat by value/maxVal, packed RGB565.
static uint16_t heatColor(int val, int maxVal) {
    if (!c_assert(maxVal > 0)) {
        return TFT_GREEN;
    }
    float t = (float)val / (float)maxVal;
    if (t > 1.0f) {
        t = 1.0f;
    }
    uint8_t r = (t < 0.5f) ? (uint8_t)(255 * t * 2) : 255;
    uint8_t g = (t < 0.5f) ? 255 : (uint8_t)(255 * (1.0f - t) * 2);
    return (uint16_t)(((r >> 3) << 11) | ((g >> 2) << 5));
}

static char sourceChar(uint8_t source) {
    return (source == SRC_WIFI) ? 'W' : 'B';
}

// ── Filter cycling ───────────────────────────────────────────────────────────
static void buildFilters() {
    g_filter_n = 0;
    g_filters[g_filter_n++] = FILTER_ALL;
    for (uint8_t c = 0; c < CLS_COUNT; ++c) {   // bound: CLS_COUNT (Rule 2)
        if (classIsPax(c)) {
            g_filters[g_filter_n++] = c;
        }
    }
    (void)c_assert(g_filter_n >= 1 && g_filter_n <= CLS_COUNT + 1);
    g_filter_idx = 0;
}

static const char* filterLabel() {
    uint8_t f = currentFilter();
    return (f == FILTER_ALL) ? "ALL" : classLabel(f);
}

static void cycleFilter(int delta) {
    if (!c_assert(g_filter_n > 0)) {
        return;
    }
    int idx = (int)g_filter_idx + delta;
    while (idx < 0) {                           // bound: one wrap (Rule 2)
        idx += (int)g_filter_n;
    }
    g_filter_idx = (size_t)idx % g_filter_n;
    g_force = true;
    g_top_sig = 0;
    g_recent_sig = 0;
}

// ── Dashboard sections ───────────────────────────────────────────────────────
static void drawDashHeader(int pax, int ble, int wifi) {
    M5.Lcd.fillRect(0, 0, SCREEN_W, PAX_REGION_H, TFT_BLACK);
    M5.Lcd.setTextSize(PAX_FONT_SZ);
    M5.Lcd.setTextColor(TFT_WHITE, TFT_BLACK);
    M5.Lcd.setCursor(DETAIL_X, PAX_PAD / 2);
    M5.Lcd.printf("%d", pax);
    M5.Lcd.setTextSize(1);
    M5.Lcd.setTextColor(TFT_CYAN, TFT_BLACK);
    M5.Lcd.setCursor(SCREEN_W - 110, 4);
    M5.Lcd.printf("BLE:%d WiFi:%d", ble, wifi);
    M5.Lcd.setTextColor(TFT_WHITE, TFT_BLACK);
}

static void drawGraph(PaxHistory& hist) {
    M5.Lcd.fillRect(GRAPH_X - 2, GRAPH_Y - 2, GRAPH_W + 4, GRAPH_H + 14, TFT_BLACK);
    int maxVal  = hist.windowMax();
    if (maxVal < 1) {
        maxVal = 1;
    }
    int step    = GRAPH_BAR_W + GRAPH_BAR_GAP;
    int maxBars = GRAPH_W / step;
    int start   = (hist.count > maxBars) ? (hist.count - maxBars) : 0;
    int nBars   = hist.count - start;
    int xOff    = GRAPH_X + GRAPH_W - (nBars * step);
    for (int i = start; i < hist.count; ++i) {   // bound: CAPACITY (Rule 2)
        int v    = hist.at(i);
        int barH = (v * GRAPH_H) / maxVal;
        if (barH < 1 && v > 0) {
            barH = 1;
        }
        int x = xOff + (i - start) * step;
        M5.Lcd.fillRect(x, GRAPH_Y + GRAPH_H - barH, GRAPH_BAR_W, barH,
                        heatColor(v, maxVal));
    }
    M5.Lcd.setTextSize(1);
    M5.Lcd.setTextColor(TFT_DARKGREY, TFT_BLACK);
    M5.Lcd.setCursor(GRAPH_X, GRAPH_Y + GRAPH_H + 2);
    M5.Lcd.printf("-%dm", hist.count);
    M5.Lcd.setCursor(GRAPH_X + GRAPH_W - 18, GRAPH_Y + GRAPH_H + 2);
    M5.Lcd.print("now");
}

static void drawDashStats(PaxHistory& hist) {
    int statsY = GRAPH_Y + GRAPH_H + 14;
    M5.Lcd.fillRect(0, statsY, SCREEN_W, 24, TFT_BLACK);
    M5.Lcd.setTextSize(1);
    M5.Lcd.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
    M5.Lcd.setCursor(DETAIL_X, statsY);
    M5.Lcd.printf("Peak:%d Avg:%ld DB:%u Drop:%u",
                  hist.peak, hist.average(), (unsigned)psSize(),
                  (unsigned)psDropped());
    unsigned long sec = millis() / 1000;
    M5.Lcd.setCursor(DETAIL_X, statsY + 10);
    M5.Lcd.printf("Up: %luh%02lum", sec / 3600, (sec % 3600) / 60);
}

static void drawFooter(const char* left, const char* mid, const char* right) {
    int y = SCREEN_H - 12;
    M5.Lcd.fillRect(0, y, SCREEN_W, 12, TFT_BLACK);
    M5.Lcd.setTextSize(1);
    M5.Lcd.setTextColor(TFT_DARKGREY, TFT_BLACK);
    M5.Lcd.setCursor(20, y);
    M5.Lcd.print(left);
    M5.Lcd.setCursor(130, y);
    M5.Lcd.print(mid);
    M5.Lcd.setCursor(268, y);
    M5.Lcd.print(right);
}

static void drawDashboard(PaxHistory& hist) {
    uint32_t sig = mix(mix((uint32_t)psPaxTotal(), (uint32_t)psPaxBle()),
                       mix((uint32_t)psPaxWifi(),
                           mix((uint32_t)hist.count, (uint32_t)hist.newest())));
    if (sig == g_dash_sig && !g_force) {
        return;
    }
    g_dash_sig = sig;
    drawDashHeader(psPaxTotal(), psPaxBle(), psPaxWifi());
    drawGraph(hist);
    drawDashStats(hist);
    drawFooter("", "[Dashboard]", "");
}

// ── Detail sections ──────────────────────────────────────────────────────────
static void drawDetailHeader(int pax) {
    M5.Lcd.fillRect(0, 0, SCREEN_W, PAX_REGION_H, TFT_BLACK);
    M5.Lcd.setTextSize(PAX_FONT_SZ);
    M5.Lcd.setTextColor(TFT_WHITE, TFT_BLACK);
    M5.Lcd.setCursor(DETAIL_X, PAX_PAD / 2);
    M5.Lcd.printf("%d", pax);
}

// Render up to `n` rows of `views`. `show_count` picks the count vs. age column.
// Returns the y just past the last row drawn.
static int drawList(const DeviceView* views, size_t n, int y, bool show_count) {
    if (!c_assert(views != nullptr)) {
        return y;
    }
    for (size_t i = 0; i < n; ++i) {             // bound: list cap (Rule 2)
        if (y + DETAIL_FONT_H > DETAIL_Y + DETAIL_H) {
            break;
        }
        char lbl[CLS_DISPLAY_W];
        truncateStr(lbl, sizeof(lbl), classLabel(views[i].cls), CLS_DISPLAY_W - 4);
        M5.Lcd.setCursor(DETAIL_X + LIST_INDENT, y);
        if (show_count) {
            M5.Lcd.printf("%u.%s %-5s (%u,%ddBm)%c", (unsigned)(i + 1),
                          views[i].mac_str, lbl, (unsigned)views[i].count,
                          views[i].rssi, sourceChar(views[i].source));
        } else {
            M5.Lcd.printf(" %s %-5s (%lus,%ddBm)%c", views[i].mac_str, lbl,
                          (unsigned long)(views[i].age_ms / 1000),
                          views[i].rssi, sourceChar(views[i].source));
        }
        y += DETAIL_FONT_H + DETAIL_SPACING;
    }
    return y;
}

static void drawDetailBody() {
    size_t top_n = 0, rec_n = 0;
    const DeviceView* top = psTopCommon(&top_n);
    const DeviceView* rec = psRecent(&rec_n);

    M5.Lcd.fillRect(0, DETAIL_Y, SCREEN_W, DETAIL_H, TFT_BLACK);
    M5.Lcd.setTextSize(DETAIL_FONT_SZ);
    int y = DETAIL_Y;

    M5.Lcd.setTextColor(TFT_YELLOW, TFT_BLACK);
    M5.Lcd.setCursor(DETAIL_X, y);
    M5.Lcd.printf("Filter: %s", filterLabel());
    y += DETAIL_FONT_H + 3;

    M5.Lcd.setTextColor(TFT_WHITE, TFT_BLACK);
    M5.Lcd.setCursor(DETAIL_X, y);
    M5.Lcd.print("Most Common:");
    y += DETAIL_FONT_H + 1;
    y = drawList(top, top_n, y, true) + 3;

    if (y < DETAIL_Y + DETAIL_H - DETAIL_FONT_H) {
        M5.Lcd.drawFastHLine(DETAIL_X, y, SCREEN_W - DETAIL_X * 2, TFT_DARKGREY);
        y += 4;
        M5.Lcd.setCursor(DETAIL_X, y);
        M5.Lcd.print("Recent Sightings:");
        y += DETAIL_FONT_H + 1;
        (void)drawList(rec, rec_n, y, false);
    }
    drawFooter("< Filt", "[Detail]", "Filt >");
}

static void drawDetail() {
    size_t top_n = 0, rec_n = 0;
    const DeviceView* top = psTopCommon(&top_n);
    const DeviceView* rec = psRecent(&rec_n);
    int      pax     = psPaxTotal();
    uint32_t top_sig = listSignature(top, top_n, false);
    uint32_t rec_sig = listSignature(rec, rec_n, true);

    if (pax != g_last_pax || g_force) {
        drawDetailHeader(pax);
        g_last_pax = pax;
    }
    if (top_sig == g_top_sig && rec_sig == g_recent_sig && !g_force) {
        return;
    }
    g_top_sig = top_sig;
    g_recent_sig = rec_sig;
    drawDetailBody();
}

// ── Touch ────────────────────────────────────────────────────────────────────
static void readTouch(bool* a, bool* b, bool* c) {
    *a = *b = *c = false;
    if (M5.Touch.getCount() <= 0) {
        return;
    }
    auto tp = M5.Touch.getDetail(0);
    if (tp.y < TOUCH_Y_MIN || tp.y > TOUCH_Y_MAX) {
        return;
    }
    if (tp.x >= TOUCH_A_XMIN && tp.x <= TOUCH_A_XMAX) {
        *a = true;
    } else if (tp.x >= TOUCH_B_XMIN && tp.x <= TOUCH_B_XMAX) {
        *b = true;
    } else if (tp.x >= TOUCH_C_XMIN && tp.x <= TOUCH_C_XMAX) {
        *c = true;
    }
}

// ── Public API ───────────────────────────────────────────────────────────────
void initDisplay() {
    M5.Lcd.setRotation(1);
    M5.Lcd.fillScreen(TFT_BLACK);
    M5.Lcd.setTextColor(TFT_WHITE, TFT_BLACK);
    buildFilters();
    g_force = true;
}

uint8_t currentFilter() {
    if (!c_assert(g_filter_idx < g_filter_n)) {
        return FILTER_ALL;
    }
    return g_filters[g_filter_idx];
}

void handleTouch() {
    bool a = false, b = false, c = false;
    readTouch(&a, &b, &c);
    bool pressA = a && !g_touchA_prev;
    bool pressB = b && !g_touchB_prev;
    bool pressC = c && !g_touchC_prev;
    g_touchA_prev = a;
    g_touchB_prev = b;
    g_touchC_prev = c;

    if (pressB) {
        int p = (int)g_page + 1;
        if (p >= (int)DisplayPage::PAGE_COUNT) {
            p = 0;
        }
        g_page = (DisplayPage)p;
        g_force = true;
    }
    if (g_page == DisplayPage::DETAIL) {
        if (pressA) {
            cycleFilter(-1);
        }
        if (pressC) {
            cycleFilter(+1);
        }
    }
}

void renderFrame(PaxHistory& history) {
    if (g_page != g_last_page) {
        M5.Lcd.fillScreen(TFT_BLACK);
        g_force = true;
        g_last_page = g_page;
    }
    if (g_page == DisplayPage::DASHBOARD) {
        drawDashboard(history);
    } else {
        drawDetail();
    }
    g_force = false;
}
