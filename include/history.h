#pragma once
// ─────────────────────────────────────────────────────────────────────────────
// PAX Counter M5Stack - PAX Count History
// ─────────────────────────────────────────────────────────────────────────────
// Circular buffer that records PAX counts at regular intervals for graphing.
// Pure C++ — no Arduino dependencies — fully testable on the host.

#include <algorithm>
#include <cstddef>

struct PaxHistory {
    static constexpr int CAPACITY = 60;   // 60 samples = 60 min at 1/min

    int  samples[CAPACITY] = {};
    int  head    = 0;     // Next write position
    int  count   = 0;     // Number of valid samples
    int  peak    = 0;     // All-time peak
    long peakAge = 0;     // Samples ago the peak was last equalled

    void push(int value) {
        samples[head] = value;
        head = (head + 1) % CAPACITY;
        if (count < CAPACITY) count++;
        if (value >= peak) {
            peak    = value;
            peakAge = 0;
        } else {
            peakAge++;
        }
    }

    // Index 0 = oldest, index count-1 = newest.
    int at(int index) const {
        int pos = (head - count + index + CAPACITY) % CAPACITY;
        return samples[pos];
    }

    int newest() const {
        return count > 0 ? at(count - 1) : 0;
    }

    int windowMax() const {
        int m = 0;
        for (int i = 0; i < count; i++)
            m = std::max(m, at(i));
        return m;
    }

    long average() const {
        if (count == 0) return 0;
        long sum = 0;
        for (int i = 0; i < count; i++) sum += at(i);
        return sum / count;
    }
};
