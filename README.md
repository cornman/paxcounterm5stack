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
- **WiFi channel hopping** — rotates through channels 1-13 to maximise
  probe request coverage.

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
- **ArduinoJson** (by Benoit Blanchon, v7+)

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

Pure-C++ tests for `truncateString`, `macToString`, and `PaxHistory` run
natively on any machine with a C++17 compiler — no board required:

```bash
g++ -std=c++17 -I . test/test_native/test_helpers.cpp -o test_pax && ./test_pax
```

## Project structure

All source files live flat in the sketch directory so Arduino IDE picks them
up automatically as tabs:

```
paxcounterm5stack/
  paxcounterm5stack.ino   Main sketch — setup() / loop()
  config.h                All tuneable constants
  types.h                 Shared types, classification labels, utilities
  device_classifier.h/cpp BLE device classification
  ble_scanner.h/cpp       ESP32 BLE scanning
  wifi_scanner.h/cpp      WiFi promiscuous-mode probe capture
  pax_store.h/cpp         Device database & SPIFFS persistence
  history.h               Circular-buffer PAX history (header-only)
  display_manager.h/cpp   Two-page touch LCD display
  test/
    test_native/
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
