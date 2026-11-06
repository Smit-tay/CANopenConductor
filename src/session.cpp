#include "canopen_conductor/session.hpp"

#include <chrono>
#include <algorithm>
#include <cstddef>
#include <cstring>
#include <new>

namespace canopen_conductor {

bool Session::configure(const LinkConfig& config) noexcept {
    if (config.interface == nullptr || config.interface[0] == '\0' || !validBitrate(config.bitrate)) {
        return false;
    }
    const std::size_t length = std::strlen(config.interface);
    if (length > kMaxInterfaceName) {
        return false;
    }
    std::memcpy(name_.data(), config.interface, length + 1);
    config_ = config;
    config_.interface = name_.data();
    return true;
}

TimePoint Session::now() noexcept {
    return TimePoint{std::chrono::duration_cast<Duration>(std::chrono::steady_clock::now().time_since_epoch())};
}

bool Session::startMaster(const MasterConfig& config) noexcept {
    if (link_.state() != LinkState::Open || master_ != nullptr) {
        return false;
    }
    const std::size_t size = Master::requiredBytes(config);
    if (size == 0) {
        return false;
    }
    void* memory = ::operator new(size, std::align_val_t{alignof(std::max_align_t)}, std::nothrow);
    if (memory == nullptr) {
        return false;
    }
    Master* master = Master::create(memory, size, config, *link_.bus());
    if (master == nullptr) {
        ::operator delete(memory, std::align_val_t{alignof(std::max_align_t)});
        return false;
    }
    masterMemory_ = memory;
    master_ = master;
    return true;
}

void Session::stopMaster() noexcept {
    if (masterMemory_ != nullptr) {
        ::operator delete(masterMemory_, std::align_val_t{alignof(std::max_align_t)});
    }
    masterMemory_ = nullptr;
    master_ = nullptr;
}

bool Session::connect(Duration timeout) noexcept {
    stopMaster();
    if (link_.state() == LinkState::Open) {
        error_ = { LinkErrc::WrongState, 0 };
        return false;
    }
    const auto start = std::chrono::steady_clock::now();
    if (!link_.open(config_)) {
        error_ = link_.lastError();
        return false;
    }
    if (std::chrono::steady_clock::now() - start > timeout) {
        link_.close();  // the answer is the timeout; a failed close adds nothing to it
        error_ = { LinkErrc::Timeout, 0 };
        return false;
    }
    error_ = {};
    return true;
}

bool Session::close() noexcept {
    stopMaster();
    const bool closed = link_.close();  // from Lost this only moves the state to Closed
    error_ = closed ? LinkError {} : link_.lastError();
    return closed;
}

SessionEvent Session::step(Duration maxWait) noexcept {
    if (link_.state() == LinkState::Open) {
        // Blocks in the read. Returns early for wake(), and reports a failure of the port once.
        Duration wait = maxWait;
        if (master_ != nullptr) {
            // End the wait when the Master next has work. At least 1 ms, so a frame that waits for room in the
            // send queue does not turn the loop into a spin.
            const TimePoint t = now();
            const TimePoint due = master_->nextDeadline(t);
            if (due != TimePoint::max()) {
                wait = (std::min)(wait, (std::max)(due - t, Duration{1000}));
            }
        }
        CanFrame frame;
        switch (link_.poll(frame, wait)) {
        case LinkEvent::Frame:
            rxFrames_.fetch_add(1, std::memory_order_relaxed);
            if (master_ != nullptr) {
                master_->onFrame(frame, now());
            }
            break;
        case LinkEvent::Lost:
            error_ = link_.lastError();
            stopMaster();
            return SessionEvent::Lost;
        case LinkEvent::Idle:
            break;
        }
        if (master_ != nullptr) {
            master_->tick(now());
        }
        return SessionEvent::None;
    }

    // Closed or lost: nothing to read. Wait, and let wake() end the wait at once.
    link_.idle(maxWait);
    return SessionEvent::None;
}

} // namespace canopen_conductor
