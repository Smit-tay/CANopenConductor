#pragma once

// Linux SocketCAN host port. Host layer: this talks to the operating system.

#include "canopen_conductor/can_bus.hpp"
#include "canopen_conductor/waker.hpp"

#include <cstddef>
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
    // False if close(2) reported an error. The socket is closed in any case.
    bool close() noexcept;

    // Makes a blocked receive() return early (RxStatus::Timeout). Not owned; set before open().
    void setWaker(Waker* waker) noexcept { waker_ = waker; }
    // The errno behind the last failed open(), send() or receive(); 0 if none.
    int osError() const noexcept { return osError_; }

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

    // Whether the interface was in listen-only mode when open() succeeded (the OS's CAN control mode).
    // Unknown before the first open().
    ListenOnly listenOnly() const noexcept { return listenOnly_; }

    // Asks the OS, over rtnetlink, whether an interface is in listen-only mode. Unknown if the interface has no CAN
    // control mode (not a CAN device, no such interface) or the OS did not answer within 200 ms. Opens and closes a
    // socket of its own; call it from the I/O thread, not from a real-time path.
    static ListenOnly queryListenOnly(const char* ifname) noexcept;

    // The reading of the OS's answer to an RTM_GETLINK request, on its own so that a test can feed it a message.
    static ListenOnly parseListenOnly(const void* message, std::size_t size) noexcept;

private:
    void onErrorFrame(std::uint32_t canId, const std::uint8_t* data) noexcept;
    void setState(BusState s) noexcept;

    int fd_ = -1;
    Waker* waker_ = nullptr;
    int osError_ = 0;
    bool lost_ = false;
    bool changed_ = false;
    BusStatus status_{};
    std::uint32_t dropped_ = 0;  // socket receive-queue drop counter
    ListenOnly listenOnly_ = ListenOnly::Unknown;
};

} // namespace canopen_conductor::linux_port
