// ─────────────────────────────────────────────────────────────────────────────
// Persistence — fixed binary record format for the known-classification cache
// ─────────────────────────────────────────────────────────────────────────────
#include "persistence.h"
#include "config.h"
#include "classification.h"
#include "c_assert.h"

static constexpr size_t HEADER_BYTES = 7;
static constexpr size_t RECORD_BYTES = 9;

// ── Little-endian byte helpers (bounded; caller guarantees room) ─────────────
static void putU16(uint8_t* b, size_t* w, uint16_t v) {
    b[(*w)++] = (uint8_t)(v & 0xFF);
    b[(*w)++] = (uint8_t)((v >> 8) & 0xFF);
}
static void putU32(uint8_t* b, size_t* w, uint32_t v) {
    for (int i = 0; i < 4; ++i) {                    // bound: 4 (Rule 2)
        b[(*w)++] = (uint8_t)((v >> (i * 8)) & 0xFF);
    }
}
static void putU64(uint8_t* b, size_t* w, uint64_t v) {
    for (int i = 0; i < 8; ++i) {                    // bound: 8 (Rule 2)
        b[(*w)++] = (uint8_t)((v >> (i * 8)) & 0xFF);
    }
}
static uint16_t getU16(const uint8_t* b, size_t* r) {
    uint16_t v = (uint16_t)b[*r] | (uint16_t)((uint16_t)b[*r + 1] << 8);
    *r += 2;
    return v;
}
static uint32_t getU32(const uint8_t* b, size_t* r) {
    uint32_t v = 0;
    for (int i = 0; i < 4; ++i) {                    // bound: 4 (Rule 2)
        v |= (uint32_t)b[*r + i] << (i * 8);
    }
    *r += 4;
    return v;
}
static uint64_t getU64(const uint8_t* b, size_t* r) {
    uint64_t v = 0;
    for (int i = 0; i < 8; ++i) {                    // bound: 8 (Rule 2)
        v |= (uint64_t)b[*r + i] << (i * 8);
    }
    *r += 8;
    return v;
}

// ── Pure encode / decode ─────────────────────────────────────────────────────
size_t persistEncode(uint8_t* buf, size_t cap,
                     const uint64_t* macs, const uint8_t* classes, size_t n) {
    if (!c_assert(buf != nullptr) || !c_assert(macs != nullptr) ||
        !c_assert(classes != nullptr)) {
        return 0;
    }
    if (!c_assert(n <= 0xFFFF)) {
        return 0;
    }
    size_t need = HEADER_BYTES + n * RECORD_BYTES;
    if (!c_assert(need <= cap)) {
        return 0;
    }
    size_t w = 0;
    putU32(buf, &w, PERSIST_MAGIC);
    buf[w++] = PERSIST_VERSION;
    putU16(buf, &w, (uint16_t)n);
    for (size_t i = 0; i < n; ++i) {                 // bound: n <= 0xFFFF (Rule 2)
        putU64(buf, &w, macs[i]);
        buf[w++] = classes[i];
    }
    (void)c_assert(w == need);
    return w;
}

bool persistDecode(const uint8_t* buf, size_t len,
                   uint64_t* macs, uint8_t* classes, size_t cap, size_t* out_n) {
    if (!c_assert(buf != nullptr) || !c_assert(macs != nullptr) ||
        !c_assert(classes != nullptr) || !c_assert(out_n != nullptr)) {
        return false;
    }
    *out_n = 0;
    if (len < HEADER_BYTES) {
        return false;
    }
    size_t r = 0;
    if (getU32(buf, &r) != PERSIST_MAGIC) {
        return false;
    }
    uint8_t ver = buf[r++];
    if (!c_assert(ver == PERSIST_VERSION)) {
        return false;
    }
    uint16_t count = getU16(buf, &r);
    size_t need = HEADER_BYTES + (size_t)count * RECORD_BYTES;
    if (len < need) {
        return false;                                // truncated / corrupt
    }
    size_t n = 0;
    for (size_t i = 0; i < count && n < cap; ++i) {  // bound: count + cap (Rule 2)
        uint64_t mac = getU64(buf, &r);
        uint8_t cls  = buf[r++];
        if (mac == 0 || !classIsValid(cls)) {
            continue;                                // skip a bad record, keep going
        }
        macs[n]    = mac;
        classes[n] = cls;
        n++;
    }
    *out_n = n;
    return true;
}

// ── SPIFFS I/O (target only) ─────────────────────────────────────────────────
#ifdef ARDUINO
#include <SPIFFS.h>
#include "known_cache.h"

// File-scope scratch so these large buffers live in static storage, not on the
// stack (Rule 3, and keeps stack usage boundable per Rules 1+3).
static uint8_t  g_io_buf[HEADER_BYTES + KNOWN_MAX * RECORD_BYTES];
static uint64_t g_io_macs[KNOWN_MAX];
static uint8_t  g_io_classes[KNOWN_MAX];

bool persistSave() {
    if (!c_assert(SPIFFS.begin(true))) {
        return false;
    }
    size_t n   = kcExport(g_io_macs, g_io_classes, KNOWN_MAX);
    size_t len = persistEncode(g_io_buf, sizeof(g_io_buf), g_io_macs, g_io_classes, n);
    if (!c_assert(len > 0)) {
        return false;
    }
    File f = SPIFFS.open(PERSISTENCE_FILE, FILE_WRITE);
    if (!c_assert(f)) {
        return false;
    }
    size_t wrote = f.write(g_io_buf, len);
    f.close();
    return c_assert(wrote == len);
}

bool persistLoad() {
    if (!c_assert(SPIFFS.begin(true))) {
        return false;
    }
    File f = SPIFFS.open(PERSISTENCE_FILE, FILE_READ);
    if (!f) {
        return true;                                 // no file yet — benign
    }
    size_t len = f.size();
    if (len > sizeof(g_io_buf)) {
        len = sizeof(g_io_buf);
    }
    size_t got = f.read(g_io_buf, len);
    f.close();
    size_t n = 0;
    if (!persistDecode(g_io_buf, got, g_io_macs, g_io_classes, KNOWN_MAX, &n)) {
        return false;
    }
    for (size_t i = 0; i < n; ++i) {                 // bound: n <= KNOWN_MAX (Rule 2)
        (void)kcImport(g_io_macs[i], g_io_classes[i]);
    }
    return true;
}
#endif  // ARDUINO
