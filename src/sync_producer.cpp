#include "canopen_conductor/sync_producer.hpp"

namespace canopen_conductor {

SyncProducer::SyncProducer(ICanBus& bus, const SyncConfig& config) noexcept : bus_(&bus), config_(config) {}

bool SyncProducer::valid(const SyncConfig& c) noexcept {
    const bool counterOk = c.counterOverflow == 0 || (c.counterOverflow >= 2 && c.counterOverflow <= 240);
    return counterOk && c.cobId <= 0x7FF && c.period >= Duration::zero();
}

TimePoint SyncProducer::nextDeadline(TimePoint now) const noexcept {
    if (config_.period <= Duration::zero() || !valid(config_)) {
        return TimePoint::max();
    }
    return started_ ? due_ : now;  // the first tick starts the grid
}

bool SyncProducer::tick(TimePoint now) noexcept {
    if (config_.period <= Duration::zero() || !valid(config_)) {
        return false;
    }
    if (!started_) {
        started_ = true;
        due_ = now + config_.period;
        return false;
    }
    if (now < due_) {
        return false;
    }

    std::uint8_t next = 0;
    CanFrame f;
    f.id = config_.cobId;
    if (config_.counterOverflow != 0) {
        next = counter_ >= config_.counterOverflow ? 1 : static_cast<std::uint8_t>(counter_ + 1);
        f.len = 1;
        f.data[0] = next;
    }
    if (!bus_->send(f)) {
        return false;  // still due; retried next tick
    }
    counter_ = next;

    due_ += config_.period;
    if (due_ <= now) {
        // Fell behind: drop the missed SYNCs, stay on the grid.
        const auto missed = (now - due_) / config_.period + 1;
        due_ += missed * config_.period;
    }
    return true;
}

} // namespace canopen_conductor
