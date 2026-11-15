#pragma once
#include <cstdint>
#include <exception>

// Estimator-owner state, independent of optional diagnostics. A selected LIO
// stage is not proof that Process2 or the LIO handler completed successfully.
struct PendingLivoPair {
    enum Phase { Idle, LioSelected, ImuProcessed, LioReady, VioSelected,
                 VioImuProcessed, Skip, Retired };
    enum Outcome { None, Initializing, EmptyCloud, InvalidOutput, IdentityChanged,
                   Aborted, VioCompleted, VioSkipped };
    Phase phase = Idle;
    Outcome outcome = None;
    uint64_t sequence = 0, retired = 0, skipped = 0;
    uint64_t source_stamp_ns = 0;
    bool source_stamp_valid = false;
    double image_time = 0, target_time = 0;
    const unsigned char *image_data = nullptr;
    bool active() const { return phase != Idle && phase != Retired; }
    void begin(double image, double target, const unsigned char *data,
               uint64_t source, bool source_valid) {
        ++sequence;
        phase = LioSelected;
        outcome = None;
        image_time = image;
        target_time = target;
        image_data = data;
        source_stamp_ns = source;
        source_stamp_valid = source_valid;
    }
    void skip(Outcome reason) { phase = Skip; outcome = reason; }
    void retire(Outcome reason) {
        if (!active()) return;
        ++retired;
        if (reason != VioCompleted) ++skipped;
        phase = Retired;
        outcome = reason;
        image_data = nullptr;
    }
};

// A failed owner operation cannot leave a reusable success token. Preserve the
// primary exception; termination/recovery stays with the existing run lifecycle.
struct PendingPairExceptionGuard {
    PendingLivoPair &pair;
    bool enabled;
    int exceptions = std::uncaught_exceptions();
    ~PendingPairExceptionGuard() {
        if (enabled && std::uncaught_exceptions() > exceptions)
            pair.retire(PendingLivoPair::Aborted);
    }
};
