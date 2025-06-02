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

// --- Configuration Constants ---
static constexpr int SCAN_TIME_SECONDS = 5;
static constexpr unsigned long WINDOW_DURATION_MS = 60 * 60 * 1000UL;
static constexpr unsigned long DATA_PROCESS_INTERVAL_MS = 5000;

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
const std::string CLS_MESHTASTIC= "Meshtastic";
const std::string CLS_HID       = "HID";
const std::string CLS_OTHER_BLE = "OtherBLE";
const std::string CLS_UNKNOWN   = "Unknown";

const std::set<std::string> PAX_RELEVANT_CLASSIFICATIONS = {
    CLS_PHONE, CLS_WATCH, CLS_AUDIO, CLS_COMPUTER, CLS_TABLET, CLS_OTHER_BLE, CLS_UNKNOWN
};

BLEScan* pBLEScan;
// static constexpr int ESP_BD_ADDR_LEN = 6; // Define if not available from headers

uint64_t bleAddressToUint64(BLEAddress& bleAddr) {
    esp_bd_addr_t* nativeAddrArrayPtr = bleAddr.getNative();
    uint64_t val = 0;
    if (nativeAddrArrayPtr != nullptr) {
        uint8_t* macBytes = *nativeAddrArrayPtr;
        for (int i = 0; i < ESP_BD_ADDR_LEN; i++) {
            val = (val << 8) | (macBytes[i]);
        }
    }
    return val;
}

struct MacActivityInfo {
    std::string mac_address_str;
    uint64_t mac_address_key;
    std::deque<unsigned long> detection_timestamps;
    size_t count_in_window = 0;
    std::string classification_label = CLS_UNKNOWN;

    MacActivityInfo() = default;
    explicit MacActivityInfo(uint64_t key, const std::string& mac_str)
        : mac_address_str(mac_str), mac_address_key(key) {}

    bool operator<(const MacActivityInfo& other) const {
        if (count_in_window != other.count_in_window) {
            return count_in_window > other.count_in_window;
        }
        return mac_address_key < other.mac_address_key;
    }
};

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
static std::vector<MacActivityInfo> last_rendered_top_common_list;
static std::vector<MacActivityInfo> last_rendered_recent_list;

// --- Function Prototypes ---
std::string classifyDevice(BLEAdvertisedDevice& device);
std::string truncateString(const std::string& str, size_t width);
void processActivityData();
void updatePaxCountDisplay();
void updateDetailDisplay(); // Renamed

void setup() {
    auto cfg = M5.config();
    M5.begin(cfg);
    Serial.begin(115200);

    M5.Lcd.setRotation(1);
    M5.Lcd.fillScreen(TFT_BLACK);
    M5.Lcd.setTextColor(TFT_WHITE, TFT_BLACK);

    updatePaxCountDisplay();
    updateDetailDisplay();  // Call the renamed display function

    BLEDevice::init("");
    pBLEScan = BLEDevice::getScan();
    pBLEScan->setActiveScan(true);
    pBLEScan->setInterval(100);
    pBLEScan->setWindow(99);
}

void loop() {
    unsigned long current_time_ms = millis();
    bool new_device_data_scanned = false;
    std::set<uint64_t> macs_sighted_this_scan_cycle;

    BLEScanResults foundDevices = pBLEScan->start(SCAN_TIME_SECONDS, false);

    if (foundDevices.getCount() > 0) {
        new_device_data_scanned = true;
        for (int i = 0; i < foundDevices.getCount(); i++) {
            BLEAdvertisedDevice device = foundDevices.getDevice(i);
            BLEAddress bleAddrNonConst = device.getAddress(); // getAddress() returns by value, so it's a modifiable copy
            uint64_t mac_key = bleAddressToUint64(bleAddrNonConst);


            if (mac_key == 0) continue;

            MacActivityInfo* activity_ptr;
            auto it = mac_activity_db.find(mac_key);

            if (it == mac_activity_db.end()) {
                std::string mac_str_for_init = bleAddrNonConst.toString();
                MacActivityInfo new_activity(mac_key, mac_str_for_init);
                new_activity.classification_label = classifyDevice(device);
                
                auto insert_result = mac_activity_db.emplace(mac_key, new_activity);
                activity_ptr = &insert_result.first->second;
            } else {
                activity_ptr = &it->second;
            }
            MacActivityInfo& activity = *activity_ptr;

            if (macs_sighted_this_scan_cycle.find(mac_key) == macs_sighted_this_scan_cycle.end()) {
                activity.detection_timestamps.push_back(current_time_ms);
                macs_sighted_this_scan_cycle.insert(mac_key);
            }
        }
        pBLEScan->clearResults();
    }

    static unsigned long last_data_processing_time_ms = 0;
    if (new_device_data_scanned || (current_time_ms - last_data_processing_time_ms > DATA_PROCESS_INTERVAL_MS)) {
        processActivityData();
        last_data_processing_time_ms = current_time_ms;
    }

    updatePaxCountDisplay();
    updateDetailDisplay(); // Call the renamed display function

    delay(200);
}

std::string classifyDevice(BLEAdvertisedDevice& device) {
    // ... (classifyDevice function remains the same)
    if (device.haveManufacturerData()) {
        std::string mfgData = device.getManufacturerData();
        if (mfgData.length() >= 2) {
            uint16_t companyId = ((uint8_t)mfgData[1] << 8) | (uint8_t)mfgData[0];
            if (companyId == 0x004C && mfgData.length() >= 4 && (uint8_t)mfgData[2] == 0x02 && (uint8_t)mfgData[3] == 0x15) {
                return CLS_IBEACON;
            }
        }
    }
    if (device.haveServiceUUID() && device.isAdvertisingService(BLEUUID((uint16_t)0xFEAA))) {
        return CLS_EDDYSTONE;
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
            case 0x02C0: return CLS_HID;
        }
    }
    if (device.haveName()) {
        std::string name = device.getName(); 
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
        if (pair.second.count_in_window > 0) { 
            all_active_macs_temp.push_back(pair.second);
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
        if (pair.second.count_in_window > 0 && !pair.second.detection_timestamps.empty()) {
            active_macs_for_recency_sort.push_back(pair.second);
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

void updateDetailDisplay() {
    bool top_common_list_changed = false;
    int num_to_render_top = sorted_top_common_macs.size();

    // --- Change detection for Top Common List ---
    if (num_to_render_top != last_rendered_top_common_list.size()) {
        top_common_list_changed = true;
    } else {
        for (int i = 0; i < num_to_render_top; ++i) {
            if (sorted_top_common_macs[i].mac_address_key != last_rendered_top_common_list[i].mac_address_key ||
                sorted_top_common_macs[i].count_in_window != last_rendered_top_common_list[i].count_in_window ||
                sorted_top_common_macs[i].classification_label != last_rendered_top_common_list[i].classification_label) {
                top_common_list_changed = true;
                break;
            }
        }
    }

    bool recent_list_changed = false;
    int num_to_render_recent = sorted_recent_macs.size();

    // --- Change detection for Recent List ---
    if (num_to_render_recent != last_rendered_recent_list.size()) {
        recent_list_changed = true;
    } else {
        for (int i = 0; i < num_to_render_recent; ++i) {
            bool item_different = false;
            if (sorted_recent_macs[i].mac_address_key != last_rendered_recent_list[i].mac_address_key ||
                sorted_recent_macs[i].classification_label != last_rendered_recent_list[i].classification_label) {
                item_different = true;
            } else {
                if (sorted_recent_macs[i].detection_timestamps.empty() != last_rendered_recent_list[i].detection_timestamps.empty()) {
                    item_different = true;
                } else if (!sorted_recent_macs[i].detection_timestamps.empty() && // Both non-empty
                           !last_rendered_recent_list[i].detection_timestamps.empty() && // Check last_rendered too
                           sorted_recent_macs[i].detection_timestamps.back() != last_rendered_recent_list[i].detection_timestamps.back()) {
                    item_different = true;
                }
            }
            if (item_different) {
                recent_list_changed = true;
                break;
            }
        }
    }

    if (top_common_list_changed || recent_list_changed) {
        M5.Lcd.fillRect(0, DETAIL_REGION_Y, SCREEN_WIDTH, DETAIL_REGION_H, TFT_BLACK);
        M5.Lcd.setTextSize(DETAIL_FONT_SIZE);

        int current_y = DETAIL_REGION_Y;

        // --- Render Top Common List ---
        M5.Lcd.setCursor(DETAIL_LIST_X_START, current_y);
        M5.Lcd.print(TITLE_TOP_COMMON.c_str());
        current_y += DETAIL_FONT_HEIGHT + SPACE_AFTER_TITLE;

        for (int i = 0; i < num_to_render_top; ++i) {
            const MacActivityInfo& info = sorted_top_common_macs[i];
            int item_y_pos = current_y + i * (DETAIL_FONT_HEIGHT + DETAIL_LIST_LINE_SPACING);
            if (item_y_pos + DETAIL_FONT_HEIGHT > DETAIL_REGION_Y + DETAIL_REGION_H) break;

            M5.Lcd.setCursor(DETAIL_LIST_X_START + LIST_ITEM_START_OFFSET, item_y_pos);
            std::string truncated_label = truncateString(info.classification_label, CLASSIFICATION_DISPLAY_WIDTH);
            
            // CORRECT printf for "Most Common" list: Rank, MAC, Class, (Count)
            M5.Lcd.printf("%1d. %s %-*s (%u)",
                          i + 1,
                          info.mac_address_str.c_str(),
                          (int)CLASSIFICATION_DISPLAY_WIDTH, truncated_label.c_str(),
                          (unsigned int)info.count_in_window);
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
            std::string truncated_label = truncateString(info.classification_label, CLASSIFICATION_DISPLAY_WIDTH);

            // CORRECT printf for "Recent Sightings" list: MAC, Class, (Time Ago s)
            unsigned long time_ago_s = 0;
            if (!info.detection_timestamps.empty()) {
                time_ago_s = (millis() - info.detection_timestamps.back()) / 1000;
            }
            M5.Lcd.printf("%s %-*s (%lus)",
                          info.mac_address_str.c_str(),
                          (int)CLASSIFICATION_DISPLAY_WIDTH, // Using full width
                          truncated_label.c_str(),
                          time_ago_s);
        }

        last_rendered_top_common_list.assign(sorted_top_common_macs.begin(), sorted_top_common_macs.begin() + num_to_render_top);
        last_rendered_recent_list.assign(sorted_recent_macs.begin(), sorted_recent_macs.begin() + num_to_render_recent);
    }
}