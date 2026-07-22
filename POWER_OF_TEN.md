# Power of Ten — Compliance Report

This firmware is written to Gerard J. Holzmann's **Power of Ten** rules for
safety-critical code (NASA/JPL, *IEEE Computer*, June 2006). The rules are
Draconian by design: each is mechanically checkable and strict enough to buy
real verification power (bounded stack, bounded execution, no heap surprises).

This document reports status per rule and — importantly — **declares every place
the rules are bent, the property that costs, and what replaces it.** A waiver
that is named is a decision; a waiver that is silent is a bug.

> Scope note: Power of Ten is a *discipline*, not a certification path. It does
> not by itself satisfy DO-178C, IEC 61508, or ISO 26262. The M5Stack libraries,
> Arduino core, and ESP-IDF underneath this sketch are **outside** its scope; the
> rules are applied to the project's own modules. Where our code meets a library
> that allocates or takes a callback, that boundary is called out below.

---

## The load-bearing rules (1–3)

### Rule 1 — Simple control flow  ·  **PASS**
No `goto`, no `setjmp`/`longjmp`, no recursion (direct or indirect). The call
graph is acyclic. The only non-terminating loops are `loop()`'s implicit cycle
and `fatalHalt()`'s deliberate spin on unrecoverable init failure — both are the
sanctioned Rule 2 exception (they *must not* terminate).

### Rule 2 — Bounded loops  ·  **PASS**
Every loop has a statically provable bound:
- Table/cache scans are bounded by `DEV_MAX`, `KNOWN_MAX`, `BLE_MAX_RESULTS`,
  `CAPTURE_BUF_SIZE` — all compile-time constants in `config.h`.
- The per-device timestamp prune (`tsPrune`) walks a ring and additionally
  carries an explicit `guard < DEV_TS_RING` counter that fires an assertion if
  exceeded — the exact idiom the paper prescribes for variable-count loops.
- Byte (de)serialization loops are bounded by 4/6/8 or by a `count` that is
  range-checked against the buffer length before the loop runs.
- `p10_audit.sh` mechanically re-checks loop-bearing functions for length; loop
  bounds themselves are reviewed at each `for`/`while`.

### Rule 3 — No dynamic memory after initialization  ·  **PASS (our code)**
This was the headline of the refactor. The previous version used
`std::map`, `std::vector`, `std::deque`, `std::string`, and ArduinoJson —
all of which allocate continuously in steady state. They are **gone** from the
project's modules and replaced with fixed static storage sized in `config.h`:

| Old (heap)                                   | New (static)                          |
|----------------------------------------------|---------------------------------------|
| `std::map<uint64_t,MacActivityInfo>` device DB | `DeviceRecord g_dev[DEV_MAX]`        |
| per-device `std::deque<unsigned long>`       | `uint32_t ts[DEV_TS_RING]` ring       |
| `std::string` labels + `std::set` membership | `Classification` enum + static tables |
| `std::map` + `std::deque` LRU cache          | `KnownEntry g_known[KNOWN_MAX]` + LRU |
| `std::vector` display lists                  | `DeviceView g_top[]/g_recent[]`       |
| ArduinoJson document + parser                | fixed binary format (`persistence.*`) |

Total steady-state footprint is a fixed ~40 KB of `.bss`, provable at compile
time. **Capacity policy:** when a table is full, a *new* entry is dropped and
counted (`dtDroppedInserts()`, shown on the dashboard as `Drop:`) — never
allocated, never silently lost.

**Boundary (declared):** the BLE stack's `getName()` / `getManufacturerData()`
return library-owned `std::string`s. `device_classifier.cpp` copies the bytes it
needs into fixed buffers (`NAME_CAP`, `MFG_CAP`) and lets the temporary die; no
dynamic data flows into the logic. The library allocates internally regardless of
our code — that is the M5/ESP layer, outside Rule 3's scope here.

---

## Style-with-teeth (4–7)

### Rule 4 — Functions ≤ ~60 lines  ·  **PASS**
`./p10_audit.sh` reports the longest function at **45 lines** (`classifyDevice`);
all 80+ functions are within budget. The old `drawDashboard` (~95) and
`drawDetailPage` (~98) were decomposed into `drawDashHeader` / `drawGraph` /
`drawDashStats` / `drawFooter` and `drawDetailHeader` / `drawList` /
`drawDetailBody`, each a page or less.

### Rule 5 — Assertion density ≥ 2 per function  ·  **PARTIAL (honest)**
`./p10_audit.sh` reports ~**1.2** assertions per function codebase-wide. The
substantive logic functions (table ops, ring math, encode/decode, ingest,
selection) carry **2–4 meaningful** `c_assert`s each — preconditions, parameter
validation, loop guards, and postconditions like `total <= size` after counting.

The average is pulled below 2.0 by ~40 trivial one-line functions: pass-through
accessors (`psPaxTotal`, `dtSize`, …) and pure switch-dispatchers
(`classifyByAppearance`) that have **no meaningful precondition to assert**. The
rule's own text says an assertion a checker can prove never fails "does not
count — `assert(true)` is not compliance." Rather than pad the metric with
vacuous asserts to reach 2.0, the density is reported honestly. Every assertion
present is a real, side-effect-free Boolean check whose failure drives an
explicit recovery (return an error / clamp to a safe value), via the paper's
value-returning `c_assert` macro (`c_assert.h`).

### Rule 6 — Smallest possible scope  ·  **PASS**
Module state is `static` at file scope, never global. `now_ms` is passed into
the pure modules rather than read from `millis()` inside them — time has one
source and the core stays testable. Display snapshots (`DeviceView`) are copies,
so the UI holds no pointer into the device table and cannot corrupt it.

### Rule 7 — Check return values; validate parameters  ·  **PASS**
- Every `esp_wifi_*` call is checked (`c_assert(... == ESP_OK)`) and failures
  propagate (`initWifiScanner` returns `false`, and the sketch continues
  BLE-only instead of pretending WiFi is up).
- `initBle()`, `persistLoad/Save`, `psIngest`, `dtIngest` return status that
  callers consume.
- Deliberately-ignored returns are `(void)`-cast with a reason: `Serial.printf`
  logging, and `psIngest` in the scanners (drops are already tracked by
  `psDropped()`).
- Public functions validate their parameters at entry (`mac_str != nullptr`,
  `classIsValid(cls)`, buffer caps) and recover rather than proceed on bad input.

---

## Toolchain discipline (8–10)

### Rule 8 — Restricted preprocessor  ·  **PASS, 2 declared waivers**
Only include guards, plain constants, and two justified constructs:
1. **`c_assert` macro** (`c_assert.h`) uses `#e` stringization and a variadic log
   call. The paper introduces this exact macro while Rule 8 forbids such
   constructs; the accepted resolution is to treat the assertion macro as
   sanctioned boilerplate. It is the only macro of its kind here.
2. **`ENABLE_WIFI_SCAN`** is the project's one feature `#ifdef`. *n* conditionals
   mean up to 2ⁿ programs to test; this is *n = 1* (two build configurations:
   BLE-only and BLE+WiFi), justified in `wifi_scanner.h`.

### Rule 9 — Restricted pointers  ·  **PASS, 1 declared waiver**
At most one level of dereference throughout; no `**p`. One waiver:
- **`esp_wifi_set_promiscuous_rx_cb(&snifferCb)`** requires a function pointer —
  mandated by ESP-IDF, unavoidable. *Property lost:* a tool can no longer
  reconstruct the full call graph through that indirection. *Replacement
  guarantee:* exactly one static callback, never reassigned, doing minimal
  ISR-simple work (copy a MAC+RSSI into a mutex-guarded fixed ring); all
  validation happens in the main-loop drain. The effective call graph stays
  knowable by inspection. Declared at the call site and here.

### Rule 10 — Zero warnings, pedantic build  ·  **PASS (native core)**
`run_tests.sh` compiles the Arduino-free core with
`-std=c++17 -Wall -Wextra -Wpedantic -Wshadow -Wconversion -Werror` — **zero
warnings**, including `-Wconversion`. The Arduino-target modules (BLE/WiFi/M5
display) can't be built off-target here; compile them in Arduino IDE with
compiler warnings set to "All" and treat any warning as a defect to fix.

Suggested next tools (Rule 10 is unenforceable without them): `clang-tidy`,
`cppcheck`, and `-fstack-usage` + call-graph analysis to turn Rules 1+3 into an
actual stack bound.

---

## How to re-check

```bash
./run_tests.sh    # builds + runs the native unit tests at max pedantic settings
./p10_audit.sh    # reports Rule 4 (function length) and Rule 5 (assert density)
```

## What is deliberately NOT claimed
- The M5/Arduino/ESP-IDF libraries are not audited and do allocate/​use callbacks
  internally. Rule 3/9 compliance is claimed for *this project's modules only*,
  with the boundaries above named.
- Rule 5 is not fully met (see above); it is reported honestly at ~1.2 rather
  than inflated.
- This is not a certification against DO-178C / IEC 61508 / ISO 26262.
