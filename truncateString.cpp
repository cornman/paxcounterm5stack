#include "truncateString.h"
#include <string>

std::string truncateString(const std::string& str, size_t width) {
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
