#pragma once
// ─────────────────────────────────────────────────────────────────────────────
// PAX Counter M5Stack - Display Manager
// ─────────────────────────────────────────────────────────────────────────────
// Two-page display driven by touch buttons:
//
//   Page 0 – DASHBOARD: large PAX count + bar-chart history + stats summary
//   Page 1 – DETAIL:    filtered device lists (most common / recently seen)
//
// Touch mapping:
//   Left  (A) — cycle filter backward  (detail page) / no-op (dashboard)
//   Mid   (B) — switch pages
//   Right (C) — cycle filter forward   (detail page) / no-op (dashboard)

#include "history.h"
#include <string>

enum class DisplayPage : uint8_t { DASHBOARD = 0, DETAIL = 1, PAGE_COUNT };

void initDisplay();
void handleTouch();                     // Read touch, update page/filter
void renderFrame(PaxHistory& history);  // Draw current page

// Filter state (shared with main for processActivityData)
const std::string& currentFilterLabel();
