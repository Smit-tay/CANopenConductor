#include "canopen_conductor/session.hpp"

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

void Session::post(Request request) noexcept {
    request_.store(request, std::memory_order_release);
    link_.wake();
}

void Session::connect() noexcept {
    post(Request::Connect);
}

void Session::disconnect() noexcept {
    post(Request::Disconnect);
}

void Session::reconnect() noexcept {
    // A Disconnect, or a Connect that has not been taken yet, must not be replaced by a reconnect.
    auto expected = Request::None;
    if (request_.compare_exchange_strong(expected, Request::Reconnect, std::memory_order_acq_rel)) {
        link_.wake();
    }
}

void Session::beginConnect(bool viaReconnect) noexcept {
    connecting_ = true;
    attempted_ = false;
    failureReported_ = false;
    viaReconnect_ = viaReconnect;
}

bool Session::close() noexcept {
    connecting_ = false;
    wanted_ = false;
    return link_.close();
}

SessionEvent Session::step(Duration maxWait) noexcept {
    switch (request_.exchange(Request::None, std::memory_order_acq_rel)) {
    case Request::Connect:
        wanted_ = true;
        if (!connecting_ && link_.state() != LinkState::Open) {
            beginConnect(false);
        }
        break;

    case Request::Reconnect:
        if (wanted_ && !connecting_ && link_.state() != LinkState::Open) {
            beginConnect(true);
        }
        break;

    case Request::Disconnect: {
        wanted_ = false;
        connecting_ = false;
        const bool wasOpen = link_.state() == LinkState::Open;
        const bool closed = link_.close();  // from Lost this only moves the state to Closed
        if (wasOpen) {
            error_ = closed ? LinkError {} : link_.lastError();
            return SessionEvent::Disconnected;
        }
        break;
    }

    case Request::None:
        break;
    }

    if (link_.state() == LinkState::Open) {
        // Blocks in the read. Returns early for a command or wake(), and reports a failure of the port once.
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

    if (connecting_) {
        // Between two attempts: pause, but let a command or wake() end the pause at once.
        if (attempted_ && link_.idle(kRetryInterval)) {
            return SessionEvent::None;
        }
        attempted_ = true;
        if (link_.open(config_)) {
            connecting_ = false;
            error_ = {};
            return SessionEvent::Connected;
        }
        // The first failure of a command is reported. After that the host already knows "not connected, and why".
        error_ = link_.lastError();
        if (!failureReported_) {
            failureReported_ = true;
            return SessionEvent::ConnectFailed;
        }
        return SessionEvent::None;
    }

    // Closed or lost, and nobody asked for anything: wait for a command.
    link_.idle(maxWait);
    return SessionEvent::None;
}

} // namespace canopen_conductor
