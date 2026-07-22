#pragma once
// ─────────────────────────────────────────────────────────────────────────────
// Display Manager — two-page touch UI
// ─────────────────────────────────────────────────────────────────────────────
//   Page 0 – DASHBOARD: large PAX count + BLE/WiFi split + history bar chart + stats
//   Page 1 – DETAIL:    filtered device lists (most common / recently seen)
//
// Touch mapping:
//   Left  (A) — previous filter (detail page only)
//   Mid   (B) — switch pages
//   Right (C) — next filter     (detail page only)

#include <cstdint>
#include "history.h"

enum class DisplayPage : uint8_t { DASHBOARD = 0, DETAIL = 1, PAGE_COUNT };

void initDisplay();
void handleTouch();                     // read touch, update page/filter
void renderFrame(PaxHistory& history);  // draw the current page

// Current filter as a Classification value, or FILTER_ALL for "no filter".
// Passed to psProcess() by the sketch.
uint8_t currentFilter();
