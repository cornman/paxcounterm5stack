#pragma once
// ─────────────────────────────────────────────────────────────────────────────
// PAX Store — facade over the device table, known cache, and persistence
// ─────────────────────────────────────────────────────────────────────────────
// The single ownership point the sketch and display talk to. It resolves weak
// classifications against the persistent cache, forwards sightings to the fixed
// device table, and drives periodic saves. No storage of its own beyond a small
// save-timer (Rule 6).

#include <cstdint>
#include <cstddef>
#include "device_table.h"   // DeviceView + result accessors

// Initialise all sub-modules and load persisted classifications. Call once.
void psInit();

// Record one sighting from a scanner. `cls` is the freshly-classified value
// (possibly weak); the facade upgrades it from the known cache and records any
// specific class back into the cache. Returns false if the device was dropped
// (table full) or arguments were invalid.
bool psIngest(uint64_t mac_key, const char* mac_str, uint8_t cls,
              int rssi, uint8_t source, bool add_timestamp, uint32_t now_ms);

// Prune + recount + rebuild selection lists for the given UI filter (a
// Classification value or FILTER_ALL).
void psProcess(uint32_t now_ms, uint8_t filter);

// Persist the known cache if it changed and the save interval has elapsed.
void psSaveIfDue(uint32_t now_ms);

// ── Results (valid after psProcess) ──────────────────────────────────────────
int      psPaxTotal();
int      psPaxBle();
int      psPaxWifi();
size_t   psSize();
uint32_t psDropped();
const DeviceView* psTopCommon(size_t* count);
const DeviceView* psRecent(size_t* count);
