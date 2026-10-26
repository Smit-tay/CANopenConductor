#pragma once

#include "canopen_conductor/can_bus.hpp"

#include <cstdint>

namespace canopen_conductor {

struct SyncConfig {
    Duration period{};                  // communication cycle period (1006h); 0 = off
    std::uint8_t counterOverflow = 0;   // 1019h: 0 = no counter, 2..240 = counter 1..value
    std::uint32_t cobId = 0x080;        // 11-bit CAN-ID (1005h)
};

// Periodic SYNC on a fixed time grid.
class SyncProducer {
public:
    SyncProducer(ICanBus& bus, const SyncConfig& config) noexcept;

    static bool valid(const SyncConfig& config) noexcept;

    // True if a SYNC was handed to the bus during this call.
    bool tick(TimePoint now) noexcept;

    // Counter of the last SYNC sent (0 when the counter is off).
    std::uint8_t counter() const noexcept { return counter_; }

    bool enabled() const noexcept { return config_.period > Duration::zero() && valid(config_); }

    // Earliest time tick() has work to do if no frame arrives first:
    // `now` if a frame is waiting for room in the send queue, a past time if
    // something is already due, TimePoint::max() if nothing is timed.
    TimePoint nextDeadline(TimePoint now) const noexcept;

private:
    ICanBus* bus_;
    SyncConfig config_;
    bool started_ = false;
    TimePoint due_{};
    std::uint8_t counter_ = 0;  // last counter sent
};

} // namespace canopen_conductor
