#include "canopen_conductor/session.hpp"

#include <chrono>
#include <cstring>

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

bool Session::connect(Duration timeout) noexcept {
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
    const bool closed = link_.close();  // from Lost this only moves the state to Closed
    error_ = closed ? LinkError {} : link_.lastError();
    return closed;
}

SessionEvent Session::step(Duration maxWait) noexcept {
    if (link_.state() == LinkState::Open) {
        // Blocks in the read. Returns early for wake(), and reports a failure of the port once.
        CanFrame frame;
        switch (link_.poll(frame, maxWait)) {
        case LinkEvent::Frame:
            rxFrames_.fetch_add(1, std::memory_order_relaxed);
            break;
        case LinkEvent::Lost:
            error_ = link_.lastError();
            return SessionEvent::Lost;
        case LinkEvent::Idle:
            break;
        }
        return SessionEvent::None;
    }

    // Closed or lost: nothing to read. Wait, and let wake() end the wait at once.
    link_.idle(maxWait);
    return SessionEvent::None;
}

} // namespace canopen_conductor
