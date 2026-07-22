#pragma once
// ─────────────────────────────────────────────────────────────────────────────
// Persistence — fixed binary record format for the known-classification cache
// ─────────────────────────────────────────────────────────────────────────────
// Replaces ArduinoJson (dynamic parser + dynamic document). On-flash layout is
// fixed and self-describing:
//
//   offset  size  field
//   0       4     magic   = PERSIST_MAGIC (little-endian)
//   4       1     version = PERSIST_VERSION
//   5       2     count   (little-endian, number of records, <= 0xFFFF)
//   7       9*N   records: [ uint64 mac (LE) ][ uint8 class ]
//
// The encode/decode functions are pure (operate on caller buffers) and are unit-
// tested natively. persistSave()/persistLoad() add the SPIFFS I/O and only exist
// on the Arduino target.

#include <cstdint>
#include <cstddef>

// Serialize N (mac, class) pairs into `buf`. Returns bytes written, or 0 if the
// buffer is too small or arguments are invalid. Needs cap >= 7 + 9*n.
size_t persistEncode(uint8_t* buf, size_t cap,
                     const uint64_t* macs, const uint8_t* classes, size_t n);

// Parse `buf` (len bytes). Writes up to `cap` pairs into macs/classes and the
// count into *out_n. Returns false on bad magic/version or a truncated buffer.
bool persistDecode(const uint8_t* buf, size_t len,
                   uint64_t* macs, uint8_t* classes, size_t cap, size_t* out_n);

#ifdef ARDUINO
// Export the known cache and write it to flash. Returns false on any I/O failure.
bool persistSave();
// Read the file (if present) and import each record into the known cache.
// Returns false only on a real error; a missing file is a benign true/no-op.
bool persistLoad();
#endif
