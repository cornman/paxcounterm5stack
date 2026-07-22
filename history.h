#pragma once
// ─────────────────────────────────────────────────────────────────────────────
// PAX Count History — fixed-size circular buffer
// ─────────────────────────────────────────────────────────────────────────────
// Records PAX counts at regular intervals for the dashboard graph. Static
// storage (Rule 3), statically-bounded loops (Rule 2), validated indices and
// assertions (Rules 5, 7). Pure C++ — unit-tested natively.

#include <cstddef>
#include "c_assert.h"

struct PaxHistory {
    static constexpr int CAPACITY = 60;   // 60 samples = 60 min at 1/min
                                          // (matches config.h HISTORY_MAX_ENTRIES)

    int  samples[CAPACITY] = {};
    int  head    = 0;     // next write position, always in [0, CAPACITY)
    int  count   = 0;     // number of valid samples, always in [0, CAPACITY]
    int  peak    = 0;     // all-time peak
    long peakAge = 0;     // samples since the peak was last equalled

    // Record one sample. Negative counts are impossible for PAX; assert and clamp.
    void push(int value) {
        if (!c_assert(value >= 0)) {
            value = 0;
        }
        if (!c_assert(head >= 0 && head < CAPACITY)) {
            head = 0;                       // recover to a sane invariant
        }
        samples[head] = value;
        head = (head + 1) % CAPACITY;
        if (count < CAPACITY) {
            count++;
        }
        if (value >= peak) {
            peak    = value;
            peakAge = 0;
        } else {
            peakAge++;
        }
    }

    // Sample by age: index 0 = oldest kept, index count-1 = newest. Returns 0 for
    // an out-of-range index rather than reading outside the live window.
    int at(int index) const {
        if (!c_assert(index >= 0 && index < count)) {
            return 0;
        }
        int pos = (head - count + index + CAPACITY) % CAPACITY;
        if (!c_assert(pos >= 0 && pos < CAPACITY)) {
            return 0;
        }
        return samples[pos];
    }

    int newest() const {
        return count > 0 ? at(count - 1) : 0;
    }

    int windowMax() const {
        int m = 0;
        for (int i = 0; i < count; i++) {   // bound: count <= CAPACITY (Rule 2)
            int v = at(i);
            if (v > m) {
                m = v;
            }
        }
        return m;
    }

    long average() const {
        if (count == 0) {
            return 0;
        }
        long sum = 0;
        for (int i = 0; i < count; i++) {   // bound: count <= CAPACITY (Rule 2)
            sum += at(i);
        }
        return sum / count;
    }
};
