# PAX Counter M5Stack

Counts nearby people by passively detecting **BLE advertisements** and
**WiFi probe requests** on an M5Stack Core2.  Displays a live PAX count,
a 60-minute history bar chart, and scrollable device detail lists on the
built-in LCD.

## Features

- **Dual-radio scanning** — BLE active scan + WiFi promiscuous-mode probe
  request capture run concurrently via ESP32 radio coexistence.
- **Device classification** — BLE devices are classified by appearance code,
  manufacturer company ID, and name keyword heuristics (Phone, Watch, Audio,
  PC/Laptop, Tablet, Sensor, Beacon, etc.).
- **Two-page touch UI**
  - *Dashboard* — large PAX count, BLE/WiFi breakdown, colour-coded history
    graph, peak / average / uptime stats.
  - *Detail list* — top-5 most common devices and top-5 recently sighted
    devices, filterable by classification.
- **Persistent classification cache** — device classifications are saved to
  SPIFFS every 15 minutes and restored on boot, with LRU eviction to cap
  memory. Stored as a compact fixed binary format (no JSON parser).
- **Serial CSV telemetry** — `CSV,millis,pax_total,pax_ble,pax_wifi,db_size`
  emitted every 30 s for easy data logging.
- **WiFi channel hopping** — rotates through channels 1-13 to maximise
  probe request coverage.
- **Written to NASA/JPL's "Power of Ten" rules** — fixed static storage (no
  heap in steady state), statically-bounded loops, small functions, assertions,
  and checked returns. See [`POWER_OF_TEN.md`](POWER_OF_TEN.md).

## Design: Power of Ten

The firmware follows Gerard J. Holzmann's [Power of Ten](https://spinroot.com/gerard/pdf/P10.pdf)
rules for safety-critical code. The core data model is **fixed-capacity static
storage** — every runtime container is a static array sized in `config.h`, so
the RAM footprint is provable at compile time and there is no heap allocation in
steady state. When a table fills, new entries are dropped and counted
(`Drop:` on the dashboard), never silently lost.

The pure, hardware-independent modules (device table, classification, known
cache, persistence, history) are unit-tested natively and compile warning-clean
under `-Wall -Wextra -Wpedantic -Wshadow -Wconversion -Werror`.
[`POWER_OF_TEN.md`](POWER_OF_TEN.md) is the full per-rule compliance report,
including the few honestly-declared waivers (e.g. the ESP-IDF promiscuous
callback that requires a function pointer).

## Hardware

| Component | Requirement |
|-----------|-------------|
| Board     | M5Stack Core2 (ESP32-based) |
| Display   | Built-in 320x240 IPS LCD |
| Touch     | Built-in capacitive touch |
| Storage   | SPIFFS (on-chip flash) |

## Arduino IDE Setup

### 1. Install the ESP32 board package

In **File > Preferences**, add this URL to *Additional Boards Manager URLs*:

```
https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
```

Then open **Tools > Board > Boards Manager**, search for **esp32** and
install.

### 2. Install required libraries

Open **Sketch > Include Library > Manage Libraries** and install:

- **M5Unified** (by M5Stack)

(ArduinoJson is no longer required — persistence uses a built-in fixed binary
format.)

### 3. Select your board

- **Tools > Board** — choose **M5Stack-Core2**
- **Tools > Port** — select the serial port for your device

### 4. Open and upload

Open `paxcounterm5stack.ino` in Arduino IDE.  All `.h` and `.cpp` files in
the same folder will appear as tabs and compile automatically.

Click **Upload** (or Ctrl+U).

### Disabling WiFi scanning

To disable WiFi probe-request capture, open `config.h` and change:

```cpp
#define ENABLE_WIFI_SCAN 1
```
to:
```cpp
#define ENABLE_WIFI_SCAN 0
```

## Running the unit tests

The pure, hardware-independent core is unit-tested natively — no board required.
Run the helper (compiles at the strictest warning level and fails on any
warning):

```bash
./run_tests.sh
```

You can also run the Power of Ten audit (function length + assertion density):

```bash
./p10_audit.sh
```

## Project structure

All source files live flat in the sketch directory so Arduino IDE picks them
up automatically as tabs. Modules are split by concern (and by testability —
the top group is pure C++ with no Arduino dependency):

```
paxcounterm5stack/
  paxcounterm5stack.ino   Main sketch — setup() / loop() orchestration
  config.h                All fixed capacities + tunables
  c_assert.h              Power of Ten Rule 5 assertion macro
  types.h                 ScanSource + bounded string helpers  (pure)
  classification.h/cpp    Classification enum + static label/PAX tables  (pure)
  history.h               Fixed circular-buffer PAX history  (pure)
  device_table.h/cpp      Fixed-capacity live device store  (pure)
  known_cache.h/cpp       Fixed MAC->class LRU cache  (pure)
  persistence.h/cpp       Fixed binary SPIFFS format (encode/decode pure)
  device_classifier.h/cpp Classify a BLE advert into a Classification
  ble_scanner.h/cpp       ESP32 BLE scanning
  wifi_scanner.h/cpp      WiFi promiscuous-mode probe capture
  pax_store.h/cpp         Facade over device_table + known_cache + persistence
  display_manager.h/cpp   Two-page touch LCD display
  run_tests.sh            Build + run native tests (pedantic, -Werror)
  p10_audit.sh            Rule 4 / Rule 5 mechanical audit
  POWER_OF_TEN.md         Per-rule compliance report + declared waivers
  test/test_native/
      test_helpers.cpp    Host-native unit tests
```

## Touch controls

| Button | Dashboard page | Detail page |
|--------|---------------|-------------|
| Left   | -             | Previous filter |
| Middle | Switch page   | Switch page |
| Right  | -             | Next filter |

## Serial output

At 115200 baud the device emits CSV rows every 30 seconds:

```
CSV,millis,pax_total,pax_ble,pax_wifi,db_size
CSV,30000,42,28,19,156
CSV,60000,45,30,20,162
```

Pipe to a file for logging: `screen /dev/ttyUSB0 115200 | tee paxlog.csv`
