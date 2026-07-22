#pragma once
// ─────────────────────────────────────────────────────────────────────────────
// Known-Classification Cache — fixed-size MAC -> class map with LRU eviction
// ─────────────────────────────────────────────────────────────────────────────
// Remembers each device's best-known Classification so a device re-seen after
// its detections aged out (or after a reboot) is re-labelled instantly. Static
// array of KNOWN_MAX entries; when full, the least-recently-used entry is evicted
// (Rule 3, no heap; Rule 2, all scans bounded by KNOWN_MAX). Pure C++, native-
// testable — persistence.cpp handles the flash I/O separately.

#include <cstdint>
#include <cstddef>

void   kcInit();

// Look up the class for `mac`. Sets *found (may be NULL). Returns the stored
// class, or CLS_UNKNOWN when absent. A hit refreshes the entry's LRU recency.
uint8_t kcLookup(uint64_t mac, bool* found);

// Insert or update mac->cls. Returns true if the cache changed (new entry or a
// different class), false if it already held that exact mapping. Only "specific"
// classes should be stored; the caller decides.
bool   kcUpdate(uint64_t mac, uint8_t cls);

size_t kcSize();

// Copy all entries into caller arrays (for saving). Writes min(size, cap) pairs
// and returns that count. macs/classes must each hold `cap` elements.
size_t kcExport(uint64_t* macs, uint8_t* classes, size_t cap);

// Load one entry (from flash). Returns false on invalid class or when full.
bool   kcImport(uint64_t mac, uint8_t cls);

bool   kcDirty();        // true if changed since the last kcClearDirty()
void   kcClearDirty();
