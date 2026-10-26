#pragma once

// EMCY producer of a Device (CiA 301 7.2.7, 1001h, 1003h). Internal header.

#include "canopen_conductor/can_bus.hpp"

#include <cstdint>

namespace canopen_conductor::detail {

struct EmcyMessage {
    std::uint16_t code = 0;
    std::uint8_t errorRegister = 0;
    std::uint8_t info[5] = {};
};

struct ActiveError {
    std::uint16_t code = 0;
    std::uint8_t bits = 0;  // error register bits this error sets
};

class EmcyProducer {
public:
    void init(ActiveError* active, std::uint8_t maxActive, std::uint32_t* history, std::uint8_t depth,
              EmcyMessage* queue, std::uint8_t queueSize) noexcept;

    // Error becomes active and an EMCY is queued. False if it was already
    // active (nothing queued) or the active table is full (sent, not kept).
    bool raise(std::uint16_t code, std::uint8_t bits, const std::uint8_t* info) noexcept;
    // Error is gone; EMCY 0000h with the remaining register is queued.
    bool clear(std::uint16_t code) noexcept;
    // One EMCY, recorded in the history, not kept active.
    void event(std::uint16_t code, std::uint8_t bits, const std::uint8_t* info) noexcept;
    // Reset: no active errors, nothing queued. The history stays.
    void reset() noexcept;
    bool active(std::uint16_t code) const noexcept;

    std::uint8_t errorRegister() const noexcept { return register_; }

    std::uint8_t historyDepth() const noexcept { return depth_; }
    std::uint8_t historyCount() const noexcept { return historyCount_; }
    std::uint32_t history(std::uint8_t i) const noexcept { return history_[i]; }  // 0 = newest
    void clearHistory() noexcept { historyCount_ = 0; }

    // Send queued messages while `allowed` and the inhibit time permits.
    // `cobId` bit 31 set: EMCY not valid, the queue is dropped.
    void tick(TimePoint now, ICanBus& bus, std::uint32_t cobId, Duration inhibit, bool allowed) noexcept;
    TimePoint nextDeadline(TimePoint now, bool allowed) const noexcept;
    std::uint32_t dropped() const noexcept { return dropped_; }

private:
    void queue(std::uint16_t code, std::uint8_t reg, const std::uint8_t* info) noexcept;
    void record(std::uint16_t code, const std::uint8_t* info) noexcept;
    void recompute() noexcept;

    ActiveError* active_ = nullptr;
    std::uint8_t maxActive_ = 0;
    std::uint8_t activeCount_ = 0;
    std::uint8_t register_ = 0;
    std::uint32_t* history_ = nullptr;
    std::uint8_t depth_ = 0;
    std::uint8_t historyCount_ = 0;
    EmcyMessage* queue_ = nullptr;
    std::uint8_t queueSize_ = 0;
    std::uint8_t head_ = 0;
    std::uint8_t count_ = 0;
    std::uint32_t dropped_ = 0;
    bool sentOnce_ = false;
    TimePoint nextAllowed_{};
};

} // namespace canopen_conductor::detail
