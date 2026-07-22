#pragma once
// ─────────────────────────────────────────────────────────────────────────────
// c_assert — Power of Ten Rule 5 assertion idiom
// ─────────────────────────────────────────────────────────────────────────────
// Holzmann's assertion macro (IEEE Computer, June 2006). Unlike <assert.h>, it
// does NOT abort: it evaluates to a bool so the caller can recover (typically by
// returning an error). Assertions here are side-effect-free Boolean tests placed
// at pre-conditions, post-conditions, and loop invariants.
//
// Usage:
//     if (!c_assert(ptr != NULL)) {
//         return false;            // caller-chosen recovery
//     }
//
// Rule 8 note (restricted preprocessor): this macro uses stringization (#e) and,
// on the target, a variadic log call. The Power of Ten paper introduces this same
// macro while its Rule 8 forbids such constructs; the accepted resolution is to
// treat the assertion macro as sanctioned boilerplate. It is the ONLY such waiver
// in this codebase. See POWER_OF_TEN.md.

#ifdef ARDUINO
  #include <Arduino.h>
  // On the target we log to Serial (a no-op if Serial is not up yet).
  #define C_ASSERT_LOG(e) \
      Serial.printf("%s,%d: assertion '%s' failed\n", __FILE__, __LINE__, #e)
#else
  #include <cstdio>
  #define C_ASSERT_LOG(e) \
      fprintf(stderr, "%s,%d: assertion '%s' failed\n", __FILE__, __LINE__, #e)
#endif

// Evaluates (e) exactly once. Returns true if it holds; on failure it logs the
// location and returns false so the caller can take an explicit recovery action.
#define c_assert(e) ((e) ? true : (C_ASSERT_LOG(e), false))
