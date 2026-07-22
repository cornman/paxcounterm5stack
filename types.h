#pragma once
// ─────────────────────────────────────────────────────────────────────────────
// Shared small types + bounded string helpers
// ─────────────────────────────────────────────────────────────────────────────
// Pure C++ (no Arduino / ESP-IDF), so these are unit-tested natively. All string
// helpers write into a caller-owned fixed buffer and never allocate (Rule 3);
// every loop has a static bound (Rule 2); parameters are validated (Rule 7).

#include <cstdint>
#include <cstddef>
#include "c_assert.h"

// Which radio saw a device. Held only in RAM, never persisted to flash.
enum ScanSource : uint8_t {
    SRC_BLE  = 0,
    SRC_WIFI = 1
};

// Format a 48-bit MAC into "aa:bb:cc:dd:ee:ff". Writes at most `cap` bytes
// including the NUL. Returns false (and writes "" if it can) when the buffer is
// too small or NULL. Needs cap >= 18.
inline bool macToStr(char* dst, size_t cap, uint64_t mac) {
    if (!c_assert(dst != nullptr)) {
        return false;
    }
    if (!c_assert(cap >= 18)) {
        if (cap > 0) {
            dst[0] = '\0';
        }
        return false;
    }
    static const char HEX[] = "0123456789abcdef";
    size_t w = 0;
    // Exactly 6 bytes, most-significant first — a fully static bound (Rule 2).
    for (int byte = 5; byte >= 0; --byte) {
        uint8_t v = (uint8_t)((mac >> (byte * 8)) & 0xFF);
        dst[w++] = HEX[(v >> 4) & 0x0F];
        dst[w++] = HEX[v & 0x0F];
        if (byte > 0) {
            dst[w++] = ':';
        }
    }
    dst[w] = '\0';
    return true;
}

// Length of a C string, bounded by `cap` so a missing NUL cannot run away
// (Rule 2). Returns cap if no NUL is found within the first cap bytes.
inline size_t boundedLen(const char* s, size_t cap) {
    if (!c_assert(s != nullptr)) {
        return 0;
    }
    size_t n = 0;
    while (n < cap && s[n] != '\0') {
        ++n;
    }
    return n;
}

// Copy `src` into `dst` truncated to at most `width` visible characters, with a
// trailing '.' when truncation occurred (mirrors the old truncateString). Always
// NUL-terminates within `cap`. Returns false on invalid arguments.
inline bool truncateStr(char* dst, size_t cap, const char* src, size_t width) {
    if (!c_assert(dst != nullptr) || !c_assert(src != nullptr)) {
        return false;
    }
    if (!c_assert(cap > 0)) {
        return false;
    }
    size_t src_len = boundedLen(src, cap + width + 1);
    size_t limit   = cap - 1;                 // room for the NUL
    size_t out_w   = 0;

    if (src_len <= width) {
        // Fits: straight copy, still bounded by the destination.
        while (out_w < limit && src[out_w] != '\0') {
            dst[out_w] = src[out_w];
            ++out_w;
        }
        dst[out_w] = '\0';
        return true;
    }
    // Truncate to `width`, reserving the final slot for '.' when width > 1.
    size_t keep = (width > 1) ? (width - 1) : width;   // width==0 -> keep 0
    if (keep > limit) {
        keep = limit;
    }
    for (size_t i = 0; i < keep; ++i) {
        dst[out_w++] = src[i];
    }
    if (width > 1 && out_w < limit) {
        dst[out_w++] = '.';
    }
    dst[out_w] = '\0';
    return true;
}
