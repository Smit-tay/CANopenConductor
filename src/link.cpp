#include "canopen_conductor/link.hpp"

#include <cctype>
#include <cstdio>
#include <cstring>

#if defined(_WIN32)
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#endif

namespace canopen_conductor {

bool validBitrate(unsigned bitrate) noexcept {
    switch (bitrate) {
    case 10000: case 20000: case 50000: case 100000: case 125000:
    case 250000: case 500000: case 800000: case 1000000:
        return true;
    default:
        return false;
    }
}

namespace {

bool startsWithNoCase(const char* text, const char* prefix) noexcept {
    for (; *prefix != '\0'; ++text, ++prefix) {
        if (std::toupper(static_cast<unsigned char>(*text)) != std::toupper(static_cast<unsigned char>(*prefix))) {
            return false;
        }
    }
    return true;
}

// A serial adapter is named "COMn", "\\.\COMn" or by a device file. Everything
// else is a network interface name.
bool isSerialName(const char* name) noexcept {
    return startsWithNoCase(name, "COM") || startsWithNoCase(name, "\\\\.\\") || name[0] == '/';
}

const char* codeName(LinkErrc code) noexcept {
    switch (code) {
    case LinkErrc::None:        return "no error";
    case LinkErrc::BadConfig:   return "bad configuration";
    case LinkErrc::NoResources: return "out of resources";
    case LinkErrc::OpenFailed:  return "cannot open the CAN interface";
    case LinkErrc::Lost:        return "CAN interface lost";
    case LinkErrc::CloseFailed: return "error while closing the CAN interface";
    }
    return "unknown error";
}

} // namespace

Link::~Link() {
    close();
}

int Link::portOsError() const noexcept {
#if defined(__linux__)
    return serial_ ? slcan_.osError() : socketCan_.osError();
#else
    return slcan_.osError();
#endif
}

bool Link::closePort() noexcept {
#if defined(__linux__)
    return serial_ ? slcan_.close() : socketCan_.close();
#else
    return slcan_.close();
#endif
}

bool Link::open(const LinkConfig& config) noexcept {
    close();
    error_ = {};

    if (config.interface == nullptr || config.interface[0] == '\0' || !validBitrate(config.bitrate)) {
        setError(LinkErrc::BadConfig, 0);
        return false;
    }
    if (!waker_.valid()) {
        setError(LinkErrc::NoResources, 0);
        return false;
    }
#if !defined(__linux__)
    // Only the serial port exists here.
    serial_ = true;
#else
    serial_ = isSerialName(config.interface);
#endif

    bool opened = false;
    if (serial_) {
        slcan_.setWaker(&waker_);
        opened = slcan_.open(config.interface, config.serialBaud, config.bitrate / 1000);
    }
#if defined(__linux__)
    else {
        socketCan_.setWaker(&waker_);
        opened = socketCan_.open(config.interface);
    }
#endif
    if (!opened) {
        const int os = portOsError();
        closePort();  // a half-open port (serial opened, adapter silent)
        setError(LinkErrc::OpenFailed, os);
        return false;
    }

#if defined(__linux__)
    active_ = serial_ ? static_cast<ICanBus*>(&slcan_) : static_cast<ICanBus*>(&socketCan_);
#else
    active_ = &slcan_;
#endif
    state_ = LinkState::Open;
    return true;
}

bool Link::close() noexcept {
    if (state_ == LinkState::Closed) {
        return true;
    }
    // From Lost the port was already released; closing it again is harmless and reports nothing.
    const bool wasOpen = state_ == LinkState::Open;
    const bool ok = closePort();
    active_ = nullptr;
    state_ = LinkState::Closed;
    if (wasOpen && !ok) {
        error_ = { LinkErrc::CloseFailed, portOsError() };
        return false;
    }
    error_ = {};
    return true;
}

bool Link::send(const CanFrame& frame) noexcept {
    return state_ == LinkState::Open && active_->send(frame);
}

LinkEvent Link::poll(CanFrame& out, Duration timeout) noexcept {
    if (state_ != LinkState::Open) {
        return LinkEvent::Idle;
    }
    switch (active_->receive(out, timeout)) {
    case RxStatus::Frame:
        return LinkEvent::Frame;
    case RxStatus::Timeout:
        return LinkEvent::Idle;
    case RxStatus::Error:
        break;
    }
    // The port failed. Keep the reason, release the port, report once.
    const int os = portOsError();
    closePort();  // ignored: the loss is the news, not the close
    active_ = nullptr;
    state_ = LinkState::Lost;
    setError(LinkErrc::Lost, os);
    return LinkEvent::Lost;
}

bool Link::busStatusChanged() const noexcept {
    if (state_ != LinkState::Open) {
        return false;
    }
#if defined(__linux__)
    return serial_ ? slcan_.busStatusChanged() : socketCan_.busStatusChanged();
#else
    return slcan_.busStatusChanged();
#endif
}

BusStatus Link::takeBusStatus() noexcept {
    if (state_ != LinkState::Open) {
        return {};
    }
#if defined(__linux__)
    return serial_ ? slcan_.takeBusStatus() : socketCan_.takeBusStatus();
#else
    return slcan_.takeBusStatus();
#endif
}

std::size_t Link::describe(const LinkError& error, char* buffer, std::size_t capacity) noexcept {
    if (capacity == 0) {
        return 0;
    }
    buffer[0] = '\0';
    int n = 0;
    if (error.osError == 0) {
        n = std::snprintf(buffer, capacity, "%s", codeName(error.code));
    } else {
        char os[160] = {};
#if defined(_WIN32)
        const DWORD got = ::FormatMessageA(FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_IGNORE_INSERTS, nullptr,
                                           static_cast<DWORD>(error.osError), 0, os, sizeof os - 1, nullptr);
        // FormatMessage ends its text with CR LF
        for (DWORD i = got; i > 0 && (os[i - 1] == '\r' || os[i - 1] == '\n' || os[i - 1] == ' '); --i) {
            os[i - 1] = '\0';
        }
#elif defined(__GLIBC__) && defined(_GNU_SOURCE)
        const char* text = ::strerror_r(error.osError, os, sizeof os);  // GNU variant: may return its own string
        std::snprintf(os, sizeof os, "%s", text);
#else
        ::strerror_r(error.osError, os, sizeof os);  // XSI variant
#endif
        n = std::snprintf(buffer, capacity, "%s: %s (%d)", codeName(error.code), os, error.osError);
    }
    if (n < 0) {
        buffer[0] = '\0';
        return 0;
    }
    return static_cast<std::size_t>(n) < capacity ? static_cast<std::size_t>(n) : capacity - 1;
}

} // namespace canopen_conductor
