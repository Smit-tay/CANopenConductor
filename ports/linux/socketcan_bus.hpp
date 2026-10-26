#pragma once

// Linux SocketCAN host port. Not part of the library.

#include "canopen_conductor/can_bus.hpp"

#include <cstdint>

namespace canopen_conductor::linux_port {

class SocketCanBus final : public ICanBus {
public:
    SocketCanBus() = default;
    ~SocketCanBus();
    SocketCanBus(const SocketCanBus&) = delete;
    SocketCanBus& operator=(const SocketCanBus&) = delete;

    // Open and bind a raw CAN socket on `ifname` (e.g. "vcan0"), with error
    // frames and receive-queue drop reporting enabled.
    bool open(const char* ifname) noexcept;
    void close() noexcept;

    // Non-blocking. False when the socket queue is full (EAGAIN/ENOBUFS)
    // or the interface is gone (then receive() returns Error).
    bool send(const CanFrame& frame) noexcept override;

    // Waits up to `timeout`. Error frames update the bus status and are
    // reported as Timeout. Error when the interface is down or gone.
    RxStatus receive(CanFrame& out, Duration timeout) noexcept override;

    // True if the bus status changed since the last takeBusStatus().
    bool busStatusChanged() const noexcept { return changed_; }
    // Current status; clears the overflow latches and the changed flag.
    BusStatus takeBusStatus() noexcept;

private:
    void onErrorFrame(std::uint32_t canId, const std::uint8_t* data) noexcept;
    void setState(BusState s) noexcept;

    int fd_ = -1;
    bool lost_ = false;
    bool changed_ = false;
    BusStatus status_{};
    std::uint32_t dropped_ = 0;  // socket receive-queue drop counter
};

} // namespace canopen_conductor::linux_port
