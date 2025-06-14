#include <M5Unified.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>

#include <string>
#include <vector>
#include <deque>
#include <map>
#include <set>
#include <algorithm>
#include <cctype>
#include <cstring>

#include <SPIFFS.h>
#include <ArduinoJson.h> // Using ArduinoJson v6 syntax

// --- Configuration Constants ---
static constexpr int SCAN_TIME_SECONDS = 5;
static constexpr unsigned long WINDOW_DURATION_MS = 60 * 60 * 1000UL;
static constexpr unsigned long DATA_PROCESS_INTERVAL_MS = 5000;

#ifndef ESP_BD_ADDR_LEN
#define ESP_BD_ADDR_LEN 6
#endif

// --- Persistence Settings ---
const char* PERSISTENCE_FILE = "/pax_classifications.json";
std::map<uint64_t, std::string> known_device_classifications;
bool classifications_changed_since_last_save = false;
unsigned long last_save_time_ms = 0;
constexpr unsigned long SAVE_INTERVAL_MS = 15 * 60 * 1000; // Save every 15 minutes if changes occurred
constexpr size_t JSON_DOC_SIZE_ESTIMATE = 2048; // Adjust based on expected number of devices

// --- Filtering Globals ---
std::vector<std::string> available_filters;
int current_filter_index = 0;
std::string current_active_filter_label = "ALL"; // Default
bool filter_display_needs_update = true;    // To force initial display of filter
bool touch_A_active_last_frame = false;
bool touch_C_active_last_frame = false;

// --- Display Configuration ---
static constexpr int TOP_N_COMMON_TO_DISPLAY = 5;   // For the "most common" list
static constexpr int RECENT_N_MACS_TO_DISPLAY = 5; // For the "recently sighted" list

// --- UI Layout Constants ---
static constexpr int PAX_COUNT_FONT_SIZE      = 6;
// ... (rest of UI layout constants remain mostly the same) ...
static constexpr int PAX_COUNT_FONT_HEIGHT    = PAX_COUNT_FONT_SIZE * 8;
static constexpr int PAX_COUNT_PADDING        = 8;
static constexpr int PAX_COUNT_REGION_H       = PAX_COUNT_FONT_HEIGHT + PAX_COUNT_PADDING;

static constexpr int DETAIL_FONT_SIZE         = 1;
static constexpr int DETAIL_FONT_HEIGHT       = DETAIL_FONT_SIZE * 8;
static constexpr int DETAIL_LIST_LINE_SPACING = 2;
static constexpr size_t CLASSIFICATION_DISPLAY_WIDTH = 9;

static constexpr int SCREEN_WIDTH             = 320;
static constexpr int SCREEN_HEIGHT            = 240;
static constexpr int DETAIL_REGION_Y          = PAX_COUNT_REGION_H;
static constexpr int DETAIL_REGION_H          = SCREEN_HEIGHT - DETAIL_REGION_Y;
static constexpr int DETAIL_LIST_X_START      = 5;
static constexpr int LIST_ITEM_START_OFFSET   = 2; // Indent for list items under titles

// --- Touch Button Zone Definitions (for M5Stack Core2) ---
constexpr int TOUCH_BTN_Y_MIN = 200; // Y-coordinate for bottom buttons
constexpr int TOUCH_BTN_Y_MAX = 240; // Screen height
constexpr int TOUCH_BTN_A_X_MIN = 0;   // Left button
constexpr int TOUCH_BTN_A_X_MAX = 106;
// constexpr int TOUCH_BTN_B_X_MIN = 107; // Middle button (unused)
// constexpr int TOUCH_BTN_B_X_MAX = 212;
constexpr int TOUCH_BTN_C_X_MIN = 213; // Right button
constexpr int TOUCH_BTN_C_X_MAX = 319; // Screen width

// Titles and Separator
static const std::string TITLE_TOP_COMMON = "Most Common:";
static const std::string TITLE_RECENTLY_SIGHTED = "Recent Sightings:";
static constexpr int SEPARATOR_LINE_Y_OFFSET = 3; // Smaller offset for the line
static constexpr int SPACE_AFTER_TITLE = 1;


// --- BLE Appearance Codes (Examples from Bluetooth SIG) ---
// ... (Appearance codes remain the same) ...
constexpr uint16_t APPEARANCE_UNKNOWN = 0x0000;
constexpr uint16_t APPEARANCE_GENERIC_PHONE = 0x0240;
constexpr uint16_t APPEARANCE_GENERIC_COMPUTER = 0x0080;
constexpr uint16_t APPEARANCE_GENERIC_WATCH = 0x00C0;
constexpr uint16_t APPEARANCE_SMART_WATCH = 0x0241;
constexpr uint16_t APPEARANCE_GENERIC_TABLET = 0x0180;
constexpr uint16_t APPEARANCE_GENERIC_HEADSET = 0x0941;
constexpr uint16_t APPEARANCE_GENERIC_HANDSFREE = 0x0942;
constexpr uint16_t APPEARANCE_GENERIC_EARBUD = 0x0947;
constexpr uint16_t APPEARANCE_GENERIC_TAG = 0x0500;
constexpr uint16_t APPEARANCE_GENERIC_THERMOMETER = 0x0300;
constexpr uint16_t APPEARANCE_GENERIC_HEART_RATE_SENSOR = 0x0340;
constexpr uint16_t APPEARANCE_GENERIC_CYCLING_COMPUTER = 0x0483;
constexpr uint16_t APPEARANCE_GENERIC_HID = 0x03C0; // Standard Generic HID

// --- Bluetooth Company Identifiers (Assigned Numbers from Bluetooth SIG) ---
constexpr uint16_t COMPANY_ID_APPLE      = 0x004C;
constexpr uint16_t COMPANY_ID_MICROSOFT  = 0x0006;
constexpr uint16_t COMPANY_ID_SAMSUNG    = 0x0075;
constexpr uint16_t COMPANY_ID_GOOGLE     = 0x00E0;
// Add more as identified

// --- iBeacon Specific Constants ---
constexpr uint8_t IBEACON_TYPE_PROXIMITY = 0x02; // Byte indicating iBeacon type
constexpr uint8_t IBEACON_DATA_LENGTH  = 0x15; // Expected data length for iBeacon

// --- Classification Labels ---
// ... (Classification labels remain the same) ...
const std::string CLS_PHONE     = "Phone";
const std::string CLS_WATCH     = "Watch";
// ... (etc.)
const std::string CLS_AUDIO     = "Audio";
const std::string CLS_COMPUTER  = "PC/Lap";
const std::string CLS_TABLET    = "Tablet";
const std::string CLS_TAG       = "Tag";
const std::string CLS_IBEACON   = "iBeacon";
const std::string CLS_EDDYSTONE = "Eddystone";
const std::string CLS_SENSOR    = "Sensor";
const std::string CLS_MESHTASTIC = "Meshtastic";
const std::string CLS_HID       = "HID";
const std::string CLS_OTHER_BLE = "OtherBLE";
const std::string CLS_UNKNOWN   = "Unknown";
const std::string CLS_SAMSUNG_DEV = "SamsungDev";
const std::string CLS_GOOGLE_DEV  = "GoogleDev";
const std::string CLS_MSFT_DEV    = "MSFTDev";

// --- Data Structures ---
struct MacActivityInfo {
    std::string mac_address_str;
    uint64_t mac_address_key;
    std::deque<unsigned long> detection_timestamps;
    size_t count_in_window = 0;
    std::string classification_label = CLS_UNKNOWN;
    int latest_rssi = 0;

    MacActivityInfo() = default;
    explicit MacActivityInfo(uint64_t key, const std::string& mac_str)
        : mac_address_str(mac_str), mac_address_key(key), count_in_window(0), classification_label(CLS_UNKNOWN), latest_rssi(0) {}

    bool operator<(const MacActivityInfo& other) const {
        if (count_in_window != other.count_in_window) {
            return count_in_window > other.count_in_window;
        }
        return mac_address_key < other.mac_address_key;
    }
};

const std::set<std::string> PAX_RELEVANT_CLASSIFICATIONS = {
    CLS_PHONE, CLS_WATCH, CLS_AUDIO, CLS_COMPUTER, CLS_TABLET, CLS_OTHER_BLE, CLS_UNKNOWN
};

BLEScan* pBLEScan;

// Converts a BLEAddress to a uint64_t for use as a map key.
// ESP_BD_ADDR_LEN is typically 6 bytes.
uint64_t bleAddressToUint64(BLEAddress& bleAddr) {
    const esp_bd_addr_t* nativeAddrPtr = bleAddr.getNative();
    uint64_t val = 0;
    if (nativeAddrPtr != nullptr) {
        for (int i = 0; i < ESP_BD_ADDR_LEN; i++) {
            val = (val << 8) | ((*nativeAddrPtr)[i]);
        }
    }
    return val;
}

// Comparator for sorting by recency (newest first)
struct CompareMacActivityByRecency {
    bool operator()(const MacActivityInfo& a, const MacActivityInfo& b) const {
        // Pre-condition: detection_timestamps should not be empty for active devices in this list.
        // This is ensured by the filtering logic before sorting.
        if (a.detection_timestamps.back() != b.detection_timestamps.back()) {
            return a.detection_timestamps.back() > b.detection_timestamps.back(); // Newest first
        }
        return a.mac_address_key < b.mac_address_key; // Tie-break by MAC
    }
};

std::map<uint64_t, MacActivityInfo> mac_activity_db;
std::vector<MacActivityInfo> sorted_top_common_macs; // For top N most common
std::vector<MacActivityInfo> sorted_recent_macs;   // For N most recent

int current_active_pax_count = 0;

static int last_displayed_pax_total = -1;
// static std::vector<MacActivityInfo> last_rendered_top_common_list; // Replaced by summary string
// static std::vector<MacActivityInfo> last_rendered_recent_list;   // Replaced by summary string
static std::string last_top_common_summary = "";
static std::string last_recent_summary = "";

// --- Function Prototypes ---
std::string classifyDevice(BLEAdvertisedDevice& device);
std::string truncateString(const std::string& str, size_t width);
void processActivityData();
void updatePaxCountDisplay();
void updateDetailDisplay(); // Updates detailed information on the display

// --- Persistence Functions ---
void loadClassifications() {
    if (!SPIFFS.begin(true)) { // true = format SPIFFS if mount failed
        Serial.println("SPIFFS Mount Failed for loading.");
        return;
    }
    File file = SPIFFS.open(PERSISTENCE_FILE, FILE_READ);
    if (!file || file.size() == 0) {
        Serial.println("Classification file not found or empty.");
        if (file) file.close();
        return;
    }

    JsonDocument doc; // Use Static for ESP32 if size is predictable
    DeserializationError error = deserializeJson(doc, file);
    file.close();

    if (error) {
        Serial.print("deserializeJson() failed: ");
        Serial.println(error.c_str());
        return;
    }

    JsonObject root = doc.as<JsonObject>();
    for (JsonPair kv : root) {
        uint64_t mac_key = 0;
        // sscanf needs a C-style string (const char*)
        if (sscanf(kv.key().c_str(), "%llu", &mac_key) == 1) {
            known_device_classifications[mac_key] = kv.value().as<std::string>();
        }
    }
    Serial.printf("Loaded %u known classifications.\n", known_device_classifications.size());
}

void saveClassifications() {
    if (!SPIFFS.begin(true)) { // Ensure SPIFFS is mounted
        Serial.println("SPIFFS Mount Failed for saving.");
        return;
    }

    JsonDocument doc; // Use Static

    for (auto const& [mac_key, classification_label] : known_device_classifications) {
        // ArduinoJson object keys must be char* or String
        String mac_key_str = String(mac_key);
        doc[mac_key_str] = classification_label;
    }

    File file = SPIFFS.open(PERSISTENCE_FILE, FILE_WRITE);
    if (!file) {
        Serial.println("Failed to open classification file for writing.");
        return;
    }

    if (serializeJson(doc, file) == 0) {
        Serial.println("Failed to write to classification file.");
    } else {
        Serial.printf("Saved %u classifications.\n", known_device_classifications.size());
        classifications_changed_since_last_save = false;
        last_save_time_ms = millis();
    }
    file.close();
}


void setup() {
    auto cfg = M5.config();
    M5.begin(cfg);
    Serial.begin(115200);

    M5.Lcd.setRotation(1);
    M5.Lcd.fillScreen(TFT_BLACK);
    M5.Lcd.setTextColor(TFT_WHITE, TFT_BLACK);

    // Populate available filters
    available_filters.push_back("ALL");
    for (const auto& classification : PAX_RELEVANT_CLASSIFICATIONS) {
        if (classification != "ALL") { // Avoid duplicates
           available_filters.push_back(classification);
        }
    }
    if (!available_filters.empty()) {
        current_active_filter_label = available_filters[current_filter_index];
    } else { // Safety for empty available_filters
        available_filters.push_back("ALL"); // Ensure "ALL" is always an option
        current_active_filter_label = "ALL";
    }

    loadClassifications(); // Load known device classifications

    updatePaxCountDisplay();
    updateDetailDisplay();  // Call the renamed display function

    BLEDevice::init("");
    pBLEScan = BLEDevice::getScan();
    if (pBLEScan == nullptr) {
        Serial.println("Error: Failed to get BLEScan object!");
        M5.Lcd.fillScreen(TFT_RED);
        M5.Lcd.setCursor(10, 10);
        M5.Lcd.setTextSize(2);
        M5.Lcd.print("BLE Init FAILED");
        while (true) { delay(1000); } // Halt
    }
    pBLEScan->setActiveScan(true);
    pBLEScan->setInterval(100);
    pBLEScan->setWindow(99);
}

void loop() {
    M5.update(); // Read button states

    // --- New Touch Button Logic for M5Stack Core2 ---
    bool btnA_wasPressed = false;
    bool btnC_wasPressed = false;

    int touch_count = M5.Touch.getCount();
    bool current_touch_A_active = false;
    bool current_touch_C_active = false;

    if (touch_count > 0) {
        lgfx::v1::touch_point_t tp;
        // Consider the first touch point for simplicity
        M5.Touch.getPoint(&tp, 0);

        if (tp.y >= TOUCH_BTN_Y_MIN && tp.y <= TOUCH_BTN_Y_MAX) {
            if (tp.x >= TOUCH_BTN_A_X_MIN && tp.x <= TOUCH_BTN_A_X_MAX) {
                current_touch_A_active = true;
            } else if (tp.x >= TOUCH_BTN_C_X_MIN && tp.x <= TOUCH_BTN_C_X_MAX) {
                current_touch_C_active = true;
            }
        }
    }

    if (current_touch_A_active && !touch_A_active_last_frame) {
        btnA_wasPressed = true;
        Serial.println("DEBUG: Touch A (Left) wasPressed detected."); // New debug
    }
    if (current_touch_C_active && !touch_C_active_last_frame) {
        btnC_wasPressed = true;
        Serial.println("DEBUG: Touch C (Right) wasPressed detected."); // New debug
    }

    touch_A_active_last_frame = current_touch_A_active;
    touch_C_active_last_frame = current_touch_C_active;

    // --- Filter Update Logic (uses btnA_wasPressed, btnC_wasPressed) ---
    bool filter_changed_by_touch = false;
    if (btnA_wasPressed) { // Cycle to next filter
        current_filter_index++;
        if (current_filter_index >= available_filters.size()) {
            current_filter_index = 0;
        }
        filter_changed_by_touch = true;
    }
    if (btnC_wasPressed) { // Cycle to previous filter
        if (!available_filters.empty()) {
           current_filter_index--;
           if (current_filter_index < 0) {
               current_filter_index = available_filters.size() - 1;
           }
        } else {
           current_filter_index = 0;
        }
        filter_changed_by_touch = true;
    }

    if (filter_changed_by_touch && !available_filters.empty()) {
        current_active_filter_label = available_filters[current_filter_index];
        filter_display_needs_update = true;
        last_top_common_summary = "";
        last_recent_summary = "";
        Serial.printf("Filter changed by touch to: %s\n", current_active_filter_label.c_str());
    }

    unsigned long current_time_ms = millis();
    bool new_device_data_scanned = false;
    std::set<uint64_t> macs_sighted_this_scan_cycle;

    BLEScanResults* pFoundDevices = pBLEScan->start(SCAN_TIME_SECONDS, false);

    if (pFoundDevices != nullptr) {
        if (pFoundDevices->getCount() > 0) {
            new_device_data_scanned = true;
            for (int i = 0; i < pFoundDevices->getCount(); i++) {
                BLEAdvertisedDevice device = pFoundDevices->getDevice(i);
                BLEAddress bleAddrNonConst = device.getAddress(); // getAddress() returns by value, so it's a modifiable copy
                uint64_t mac_key = bleAddressToUint64(bleAddrNonConst);


            if (mac_key == 0) continue;

            MacActivityInfo* activity_ptr;
            auto it = mac_activity_db.find(mac_key);

            if (it == mac_activity_db.end()) { // Device first time seen in this session
                std::string mac_str_for_init = bleAddrNonConst.toString().c_str();
                MacActivityInfo new_activity(mac_key, mac_str_for_init);

                std::string initial_classification = classifyDevice(device);
                // If classification is vague, try to use a known one from persistence
                if (initial_classification == CLS_UNKNOWN || initial_classification == CLS_OTHER_BLE) {
                    auto known_it = known_device_classifications.find(mac_key);
                    if (known_it != known_device_classifications.end()) {
                        initial_classification = known_it->second;
                    }
                }
                new_activity.classification_label = initial_classification;

                auto insert_result = mac_activity_db.emplace(mac_key, new_activity);
                activity_ptr = &insert_result.first->second;
            } else { // Device already in DB for this session
                activity_ptr = &it->second;
                // Try to re-classify if new info is better
                std::string new_label_from_scan = classifyDevice(device);
                if (new_label_from_scan != CLS_UNKNOWN && new_label_from_scan != CLS_OTHER_BLE) { // Only update if new scan gives specific info
                    if (activity_ptr->classification_label != new_label_from_scan) {
                         activity_ptr->classification_label = new_label_from_scan;
                    }
                }
            }
            // MacActivityInfo& activity = *activity_ptr; // activity_ptr is already set by this point
            activity_ptr->latest_rssi = device.getRSSI(); // Update with the latest RSSI reading

            // Persist new/updated classification if it's meaningful
            if (activity_ptr->classification_label != CLS_UNKNOWN && activity_ptr->classification_label != CLS_OTHER_BLE) {
                auto known_it = known_device_classifications.find(mac_key);
                if (known_it == known_device_classifications.end() || known_it->second != activity_ptr->classification_label) {
                    known_device_classifications[mac_key] = activity_ptr->classification_label;
                    classifications_changed_since_last_save = true;
                    // Serial.printf("Staged classification for %s: %s\n", activity_ptr->mac_address_str.c_str(), activity_ptr->classification_label.c_str()); // Optional: for debugging
                }
            }

            // Record timestamp for this sighting in this scan cycle
            if (macs_sighted_this_scan_cycle.find(mac_key) == macs_sighted_this_scan_cycle.end()) {
                activity_ptr->detection_timestamps.push_back(millis()); // Use activity_ptr here
                macs_sighted_this_scan_cycle.insert(mac_key);
            }
        }
        pBLEScan->clearResults(); // Clear results from the BLEScan object
        }
    } else {
        Serial.println("Error: pBLEScan->start() returned nullptr, skipping scan processing.");
        // new_device_data_scanned will remain false
    }

    static unsigned long last_data_processing_time_ms = 0;
    if (new_device_data_scanned || (current_time_ms - last_data_processing_time_ms > DATA_PROCESS_INTERVAL_MS)) {
        processActivityData();
        last_data_processing_time_ms = current_time_ms;
    }

    updatePaxCountDisplay();
    updateDetailDisplay(); // Call the renamed display function

    // Periodic saving of classifications
    unsigned long current_time_ms_for_save = millis();
    if (classifications_changed_since_last_save && (current_time_ms_for_save - last_save_time_ms > SAVE_INTERVAL_MS)) {
        saveClassifications();
    }

    delay(200);
}

std::string classifyDevice(BLEAdvertisedDevice& device) {
    // ... (classifyDevice function remains the same)
    if (device.haveManufacturerData()) {
        std::string mfgData = device.getManufacturerData().c_str();
        if (mfgData.length() >= 2) {
            uint16_t companyId = ((uint8_t)mfgData[1] << 8) | (uint8_t)mfgData[0];
            if (companyId == COMPANY_ID_APPLE &&
                mfgData.length() >= 4 && // Basic check for iBeacon structure
                (uint8_t)mfgData[2] == IBEACON_TYPE_PROXIMITY &&
                (uint8_t)mfgData[3] == IBEACON_DATA_LENGTH) {
                return CLS_IBEACON;
            }
        }
    }
    if (device.haveServiceUUID() && device.isAdvertisingService(BLEUUID((uint16_t)0xFEAA))) {
        return CLS_EDDYSTONE;
    }

    if (device.haveManufacturerData()) {
        std::string mfgData = device.getManufacturerData().c_str();
        if (mfgData.length() >= 2) { // Need at least 2 bytes for Company ID
            uint16_t companyId = ((uint8_t)mfgData[1] << 8) | (uint8_t)mfgData[0]; // Little Endian
            switch (companyId) {
                case COMPANY_ID_MICROSOFT:
                    return CLS_MSFT_DEV;
                // case COMPANY_ID_APPLE: // Apple - Already handled by iBeacon check more specifically.
                case COMPANY_ID_SAMSUNG:
                    return CLS_SAMSUNG_DEV;
                case COMPANY_ID_GOOGLE:
                    return CLS_GOOGLE_DEV;
                // Add more company IDs here if desired
            }
        }
    }

    if (device.haveAppearance()) {
        uint16_t appearance = device.getAppearance();
        switch (appearance) {
            case APPEARANCE_GENERIC_PHONE: return CLS_PHONE;
            case APPEARANCE_SMART_WATCH: return CLS_WATCH;
            case APPEARANCE_GENERIC_WATCH: return CLS_WATCH;
            case APPEARANCE_GENERIC_COMPUTER: return CLS_COMPUTER;
            case APPEARANCE_GENERIC_TABLET: return CLS_TABLET;
            case APPEARANCE_GENERIC_HEADSET: return CLS_AUDIO;
            case APPEARANCE_GENERIC_EARBUD: return CLS_AUDIO;
            case APPEARANCE_GENERIC_HANDSFREE: return CLS_AUDIO;
            case APPEARANCE_GENERIC_TAG: return CLS_TAG;
            case APPEARANCE_GENERIC_THERMOMETER: 
            case APPEARANCE_GENERIC_HEART_RATE_SENSOR:
            case APPEARANCE_GENERIC_CYCLING_COMPUTER: return CLS_SENSOR;
            case APPEARANCE_GENERIC_HID: return CLS_HID;
        }
    }
    if (device.haveName()) {
        std::string name = device.getName().c_str();
        std::string lower_name = name;
        std::transform(lower_name.begin(), lower_name.end(), lower_name.begin(), 
                       [](unsigned char c){ return std::tolower(c); });

        if (lower_name.find("meshtastic") != std::string::npos) return CLS_MESHTASTIC;
        if (!device.haveAppearance()) {
            if (lower_name.find("beacon") != std::string::npos) return CLS_IBEACON;
            if (lower_name.find("sensor") != std::string::npos) return CLS_SENSOR;
            if (lower_name.find("tag") != std::string::npos || lower_name.find("tile") != std::string::npos) return CLS_TAG;
            if (lower_name.find("phone") != std::string::npos) return CLS_PHONE;
            if (lower_name.find("watch") != std::string::npos) return CLS_WATCH;
            if (lower_name.find("airpods") != std::string::npos || 
                lower_name.find("galaxy buds") != std::string::npos || 
                lower_name.find("headset") != std::string::npos || 
                lower_name.find("earbuds") != std::string::npos) return CLS_AUDIO;
            if (lower_name.find("galaxy watch") != std::string::npos) return CLS_WATCH; // More specific for Samsung watches
            if (lower_name.find("pixel buds") != std::string::npos) return CLS_AUDIO;   // Google audio
            if (lower_name.find("surface") != std::string::npos) { // Could be various MSFT devices
                if (lower_name.find("headphones") != std::string::npos) return CLS_AUDIO;
                return CLS_MSFT_DEV; // Generic MSFT if name contains "surface"
            }
        }
    }
    if (device.haveName() || device.haveServiceUUID() || device.haveManufacturerData() || device.haveAppearance()) {
        return CLS_OTHER_BLE;
    }
    return CLS_UNKNOWN;
}

std::string truncateString(const std::string& str, size_t width) {
    // ... (truncateString function remains the same)
    if (str.length() > width) {
        if (width > 1) {
            return str.substr(0, width - 1) + ".";
        } else if (width == 1) {
            return str.substr(0, 1);
        } else {
            return ""; 
        }
    }
    return str;
}

void processActivityData() {
    unsigned long current_time_ms = millis();
    std::vector<uint64_t> mac_keys_to_prune;
    int pax_relevant_device_count = 0;

    for (auto it = mac_activity_db.begin(); it != mac_activity_db.end(); ++it) {
        MacActivityInfo& activity = it->second;
        while (!activity.detection_timestamps.empty() &&
               (current_time_ms - activity.detection_timestamps.front() > WINDOW_DURATION_MS)) {
            activity.detection_timestamps.pop_front();
        }
        activity.count_in_window = activity.detection_timestamps.size();

        if (activity.count_in_window == 0) {
            mac_keys_to_prune.push_back(it->first);
        } else {
            if (PAX_RELEVANT_CLASSIFICATIONS.count(activity.classification_label)) {
                pax_relevant_device_count++;
            }
        }
    }

    for (const uint64_t mac_key : mac_keys_to_prune) {
        mac_activity_db.erase(mac_key);
    }
    current_active_pax_count = pax_relevant_device_count;

    // --- Prepare Top N Common MACs ---
    std::vector<MacActivityInfo> all_active_macs_temp;
    all_active_macs_temp.reserve(mac_activity_db.size());
    for (const auto& pair : mac_activity_db) {
        if (pair.second.count_in_window > 0) { // Only consider active devices
            if (current_active_filter_label == "ALL" || pair.second.classification_label == current_active_filter_label) {
                all_active_macs_temp.push_back(pair.second);
            }
        }
    }
    
    sorted_top_common_macs.clear();
    if (!all_active_macs_temp.empty()) {
        size_t actual_top_common_n = std::min(all_active_macs_temp.size(), (size_t)TOP_N_COMMON_TO_DISPLAY);
        sorted_top_common_macs.resize(actual_top_common_n); 
        if (actual_top_common_n > 0) {
            std::partial_sort_copy(all_active_macs_temp.begin(), all_active_macs_temp.end(),
                                   sorted_top_common_macs.begin(), sorted_top_common_macs.end());
        }
    }

    // --- Prepare Recently Sighted MACs ---
    std::vector<MacActivityInfo> active_macs_for_recency_sort;
    active_macs_for_recency_sort.reserve(mac_activity_db.size());
    for (const auto& pair : mac_activity_db) {
        // Include if active and has at least one timestamp (to get .back())
        if (pair.second.count_in_window > 0 && !pair.second.detection_timestamps.empty()) { // Active and has timestamps
             if (current_active_filter_label == "ALL" || pair.second.classification_label == current_active_filter_label) {
                active_macs_for_recency_sort.push_back(pair.second);
            }
        }
    }
    
    std::sort(active_macs_for_recency_sort.begin(), active_macs_for_recency_sort.end(), CompareMacActivityByRecency());
    
    sorted_recent_macs.clear();
    size_t actual_recent_n = std::min(active_macs_for_recency_sort.size(), (size_t)RECENT_N_MACS_TO_DISPLAY);
    if (actual_recent_n > 0) {
        sorted_recent_macs.assign(active_macs_for_recency_sort.begin(), 
                                  active_macs_for_recency_sort.begin() + actual_recent_n);
    }
}

void updatePaxCountDisplay() {
    // ... (updatePaxCountDisplay function remains the same) ...
    if (current_active_pax_count != last_displayed_pax_total) {
        M5.Lcd.fillRect(0, 0, SCREEN_WIDTH, PAX_COUNT_REGION_H, TFT_BLACK);
        M5.Lcd.setTextSize(PAX_COUNT_FONT_SIZE);
        M5.Lcd.setCursor(DETAIL_LIST_X_START, PAX_COUNT_PADDING / 2);
        M5.Lcd.printf("%d", current_active_pax_count);
        last_displayed_pax_total = current_active_pax_count;
    }
}

std::string generate_list_summary(const std::vector<MacActivityInfo>& mac_list, bool is_top_common_style) {
    std::string summary;
    summary.reserve(mac_list.size() * 50); // Adjusted reserve capacity
    for (const auto& info : mac_list) {
        summary += std::to_string(info.mac_address_key);
        summary += ';';
        summary += info.classification_label;
        summary += ';';
        if (is_top_common_style) {
            summary += std::to_string(info.count_in_window);
        } else { // Recent style
            if (!info.detection_timestamps.empty()) {
                summary += std::to_string(info.detection_timestamps.back());
            } else {
                summary += "N/A"; // Or some placeholder if empty
            }
        }
        summary += ';'; // Separator before RSSI
        summary += std::to_string(info.latest_rssi);
        summary += '|'; // Item separator
    }
    return summary;
}

void updateDetailDisplay() {
    // Generate compact string summaries of the lists for efficient change detection
    std::string current_top_common_summary = generate_list_summary(sorted_top_common_macs, true);
    std::string current_recent_summary = generate_list_summary(sorted_recent_macs, false);

    bool data_lists_changed = false;
    if (current_top_common_summary != last_top_common_summary) {
        last_top_common_summary = current_top_common_summary;
        data_lists_changed = true;
    }
    if (current_recent_summary != last_recent_summary) {
        last_recent_summary = current_recent_summary;
        data_lists_changed = true;
    }

    if (data_lists_changed || filter_display_needs_update) {
        M5.Lcd.fillRect(0, DETAIL_REGION_Y, SCREEN_WIDTH, DETAIL_REGION_H, TFT_BLACK); // Clear entire detail area
        M5.Lcd.setTextSize(DETAIL_FONT_SIZE);

        int current_y = DETAIL_REGION_Y;

        // Display Current Filter
        M5.Lcd.setCursor(DETAIL_LIST_X_START, current_y);
        M5.Lcd.setTextColor(TFT_YELLOW, TFT_BLACK); // Make filter stand out
        M5.Lcd.printf("Filter: %s", current_active_filter_label.c_str());
        current_y += DETAIL_FONT_HEIGHT + SPACE_AFTER_TITLE + 2; // Add some space
        M5.Lcd.setTextColor(TFT_WHITE, TFT_BLACK); // Reset color for other text

        filter_display_needs_update = false; // Reset flag AFTER using it

        int num_to_render_top = sorted_top_common_macs.size();
        int num_to_render_recent = sorted_recent_macs.size();

        // --- Render Top Common List ---
        M5.Lcd.setCursor(DETAIL_LIST_X_START, current_y);
        M5.Lcd.print(TITLE_TOP_COMMON.c_str());
        current_y += DETAIL_FONT_HEIGHT + SPACE_AFTER_TITLE;

        for (int i = 0; i < num_to_render_top; ++i) {
            const MacActivityInfo& info = sorted_top_common_macs[i];
            int item_y_pos = current_y + i * (DETAIL_FONT_HEIGHT + DETAIL_LIST_LINE_SPACING);
            if (item_y_pos + DETAIL_FONT_HEIGHT > DETAIL_REGION_Y + DETAIL_REGION_H) break;

            M5.Lcd.setCursor(DETAIL_LIST_X_START + LIST_ITEM_START_OFFSET, item_y_pos);
            std::string truncated_label = truncateString(info.classification_label, CLASSIFICATION_DISPLAY_WIDTH - 4);
            
            M5.Lcd.printf("%1d. %s %-*s (%u,%ddBm)",
                          i + 1,
                          info.mac_address_str.c_str(),
                          (int)(CLASSIFICATION_DISPLAY_WIDTH - 4),
                          truncated_label.c_str(),
                          (unsigned int)info.count_in_window,
                          info.latest_rssi);
        }
        current_y += num_to_render_top * (DETAIL_FONT_HEIGHT + DETAIL_LIST_LINE_SPACING);
        current_y += SEPARATOR_LINE_Y_OFFSET;

        // --- Draw Separator Line ---
        if (current_y < DETAIL_REGION_Y + DETAIL_REGION_H - (DETAIL_FONT_HEIGHT + SPACE_AFTER_TITLE) ) {
            M5.Lcd.drawFastHLine(DETAIL_LIST_X_START, current_y, SCREEN_WIDTH - (2 * DETAIL_LIST_X_START), TFT_DARKGREY);
            current_y += 1 + SEPARATOR_LINE_Y_OFFSET; // 1 for line height
        }

        // --- Render Recently Sighted List ---
        if (current_y < DETAIL_REGION_Y + DETAIL_REGION_H - DETAIL_FONT_HEIGHT) {
            M5.Lcd.setCursor(DETAIL_LIST_X_START, current_y);
            M5.Lcd.print(TITLE_RECENTLY_SIGHTED.c_str());
            current_y += DETAIL_FONT_HEIGHT + SPACE_AFTER_TITLE;
        }

        for (int i = 0; i < num_to_render_recent; ++i) {
            const MacActivityInfo& info = sorted_recent_macs[i];
            int item_y_pos = current_y + i * (DETAIL_FONT_HEIGHT + DETAIL_LIST_LINE_SPACING);
            if (item_y_pos + DETAIL_FONT_HEIGHT > DETAIL_REGION_Y + DETAIL_REGION_H) break;

            M5.Lcd.setCursor(DETAIL_LIST_X_START + LIST_ITEM_START_OFFSET, item_y_pos);
            std::string truncated_label_rec = truncateString(info.classification_label, CLASSIFICATION_DISPLAY_WIDTH - 4);

            unsigned long time_ago_s = 0;
            if (!info.detection_timestamps.empty()) {
                time_ago_s = (millis() - info.detection_timestamps.back()) / 1000;
            }
            M5.Lcd.printf("%s %-*s (%lus,%ddBm)",
                          info.mac_address_str.c_str(),
                          (int)(CLASSIFICATION_DISPLAY_WIDTH - 4),
                          truncated_label_rec.c_str(),
                          time_ago_s,
                          info.latest_rssi);
        }
        // No longer need to assign to last_rendered_... lists
    }
}