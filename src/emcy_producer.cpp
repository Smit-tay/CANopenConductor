#include "emcy_producer.hpp"

namespace canopen_conductor::detail {

void EmcyProducer::init(ActiveError* active, std::uint8_t maxActive, std::uint32_t* history, std::uint8_t depth,
                        EmcyMessage* queue, std::uint8_t queueSize) noexcept {
    active_ = active;
    maxActive_ = maxActive;
    history_ = history;
    depth_ = depth;
    historyCount_ = 0;
    queue_ = queue;
    queueSize_ = queueSize;
    reset();
}

void EmcyProducer::reset() noexcept {
    activeCount_ = 0;
    register_ = 0;
    head_ = 0;
    count_ = 0;
}

bool EmcyProducer::active(std::uint16_t code) const noexcept {
    for (std::uint8_t i = 0; i < activeCount_; ++i) {
        if (active_[i].code == code) {
            return true;
        }
    }
    return false;
}

void EmcyProducer::recompute() noexcept {
    std::uint8_t reg = 0;
    for (std::uint8_t i = 0; i < activeCount_; ++i) {
        reg = static_cast<std::uint8_t>(reg | active_[i].bits);
    }
    register_ = activeCount_ > 0 ? static_cast<std::uint8_t>(reg | 0x01) : 0;  // generic error bit
}

void EmcyProducer::record(std::uint16_t code, const std::uint8_t* info) noexcept {
    if (depth_ == 0) {
        return;
    }
    const std::uint32_t extra = info != nullptr ? static_cast<std::uint32_t>(info[0] | (info[1] << 8)) : 0;
    const std::uint8_t keep = historyCount_ < depth_ ? historyCount_ : static_cast<std::uint8_t>(depth_ - 1);
    for (std::uint8_t i = keep; i > 0; --i) {
        history_[i] = history_[i - 1];  // newest at the front
    }
    history_[0] = code | (extra << 16);
    historyCount_ = static_cast<std::uint8_t>(keep + 1);
}

void EmcyProducer::queue(std::uint16_t code, std::uint8_t reg, const std::uint8_t* info) noexcept {
    if (count_ >= queueSize_) {
        ++dropped_;
        return;
    }
    EmcyMessage& m = queue_[(head_ + count_) % queueSize_];
    m = EmcyMessage{};
    m.code = code;
    m.errorRegister = reg;
    for (int i = 0; i < 5 && info != nullptr; ++i) {
        m.info[i] = info[i];
    }
    ++count_;
}

bool EmcyProducer::raise(std::uint16_t code, std::uint8_t bits, const std::uint8_t* info) noexcept {
    if (active(code)) {
        return false;
    }
    const bool kept = activeCount_ < maxActive_;
    if (kept) {
        active_[activeCount_++] = {code, bits};
        recompute();
    }
    record(code, info);
    queue(code, static_cast<std::uint8_t>(register_ | bits | 0x01), info);
    return kept;
}

bool EmcyProducer::clear(std::uint16_t code) noexcept {
    for (std::uint8_t i = 0; i < activeCount_; ++i) {
        if (active_[i].code == code) {
            active_[i] = active_[--activeCount_];
            recompute();
            queue(0x0000, register_, nullptr);
            return true;
        }
    }
    return false;
}

void EmcyProducer::event(std::uint16_t code, std::uint8_t bits, const std::uint8_t* info) noexcept {
    record(code, info);
    queue(code, static_cast<std::uint8_t>(register_ | bits | 0x01), info);
}

void EmcyProducer::tick(TimePoint now, ICanBus& bus, std::uint32_t cobId, Duration inhibit, bool allowed) noexcept {
    if ((cobId & 0x80000000u) != 0) {
        count_ = 0;  // EMCY not valid: nothing is sent
        return;
    }
    while (allowed && count_ > 0 && (!sentOnce_ || now >= nextAllowed_)) {
        const EmcyMessage& m = queue_[head_];
        CanFrame f;
        f.id = cobId & 0x7FF;
        f.len = 8;
        f.data[0] = static_cast<std::uint8_t>(m.code);
        f.data[1] = static_cast<std::uint8_t>(m.code >> 8);
        f.data[2] = m.errorRegister;
        for (int i = 0; i < 5; ++i) {
            f.data[3 + i] = m.info[i];
        }
        if (!bus.send(f)) {
            return;  // retried on the next tick
        }
        head_ = static_cast<std::uint8_t>((head_ + 1) % queueSize_);
        --count_;
        sentOnce_ = true;
        nextAllowed_ = now + inhibit;
    }
}

TimePoint EmcyProducer::nextDeadline(TimePoint now, bool allowed) const noexcept {
    if (!allowed || count_ == 0) {
        return TimePoint::max();
    }
    return (!sentOnce_ || nextAllowed_ <= now) ? now : nextAllowed_;
}

} // namespace canopen_conductor::detail
