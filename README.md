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
  memory.
- **Serial CSV telemetry** — `CSV,millis,pax_total,pax_ble,pax_wifi,db_size`
  emitted every 30 s for easy data logging.
- **WiFi channel hopping** — rotates through channels 1–13 to maximise
  probe request coverage.

## Hardware

| Component | Requirement |
|-----------|-------------|
| Board     | M5Stack Core2 (ESP32-based) |
| Display   | Built-in 320x240 IPS LCD |
| Touch     | Built-in capacitive touch |
| Storage   | SPIFFS (on-chip flash) |

## Build

This is a [PlatformIO](https://platformio.org/) project.

```bash
# Build firmware
pio run

# Upload to connected M5Stack Core2
pio run -t upload

# Monitor serial output
pio device monitor
```

### Disabling WiFi scanning

WiFi probe-request scanning can be turned off at compile time:

```ini
# In platformio.ini, change:
build_flags = ... -DENABLE_WIFI_SCAN=0
```

## Running tests

Pure-C++ unit tests (truncateString, macToString, PaxHistory, etc.) run
natively on the host — no board required:

```bash
# PlatformIO
pio test -e native

# Or raw g++
g++ -std=c++17 -I include test/test_native/test_helpers.cpp -o test_pax && ./test_pax
```

## Project structure

```
include/
  config.h              All tuneable constants
  types.h               Shared types, classification labels, utility functions
  device_classifier.h   BLE device classification (header)
  ble_scanner.h         BLE scan management (header)
  wifi_scanner.h        WiFi probe-request capture (header)
  pax_store.h           Device database & SPIFFS persistence (header)
  history.h             Circular-buffer PAX history for graphing
  display_manager.h     Two-page touch display (header)
src/
  main.cpp              setup() / loop() orchestration
  device_classifier.cpp Classification logic
  ble_scanner.cpp       ESP32 BLE scanning
  wifi_scanner.cpp      ESP32 WiFi promiscuous mode
  pax_store.cpp         Device DB, window processing, persistence
  display_manager.cpp   LCD rendering, touch handling
test/
  test_native/
    test_helpers.cpp    Host-native unit tests
```

## Touch controls

| Button | Dashboard page | Detail page |
|--------|---------------|-------------|
| Left   | —             | Previous filter |
| Middle | Switch page   | Switch page |
| Right  | —             | Next filter |

## Serial output

At 115200 baud, the device emits CSV rows every 30 seconds:

```
CSV,millis,pax_total,pax_ble,pax_wifi,db_size
CSV,30000,42,28,19,156
CSV,60000,45,30,20,162
```
