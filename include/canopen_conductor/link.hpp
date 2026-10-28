#pragma once

// Link: one CAN connection, whichever port carries it. Host layer: this talks to the operating system.
//
// The host asks for an interface by name and a bit rate. Link picks the port,
// opens and closes it, tells open, closed and lost apart, and says why a call
// failed. It starts no thread and keeps no clock: the host calls poll() from
// its own I/O thread, and wake() from any thread to get poll() back early.
//
// Threads: open(), close(), send(), poll(), state(), lastError() and
// takeBusStatus() belong to one thread (the I/O thread). Only wake() may be
// called from another one, at any time.

#include "canopen_conductor/can_bus.hpp"
#include "canopen_conductor/frame.hpp"
#include "canopen_conductor/time.hpp"
#include "canopen_conductor/waker.hpp"
#include "canopen_conductor/slcan_bus.hpp"

#if defined(__linux__)
#include "canopen_conductor/socketcan_bus.hpp"
#endif

#include <cstddef>

namespace canopen_conductor {

enum class LinkState : std::uint8_t {
    Closed,  // never opened, or closed by the host
    Open,    // working
    Lost,    // was open, the port failed. It has been released; open() again to recover.
};

enum class LinkErrc : std::uint8_t {
    None,
    BadConfig,    // empty interface, or a bit rate CANopen/slcan has no code for
    NoResources,  // the wake-up could not be created
    OpenFailed,   // the interface does not exist, is busy, or refused the settings
    Lost,         // the port failed while open
    CloseFailed,  // the OS reported an error while closing. The link is closed anyway.
};

struct LinkError {
    LinkErrc code = LinkErrc::None;
    int osError = 0;  // errno, or GetLastError() on Windows; 0 if the OS gave none
};

// The result of poll().
enum class LinkEvent : std::uint8_t {
    Frame,  // `out` holds a frame
    Idle,   // timeout, wake, or nothing to wait for (the link is not open)
    Lost,   // the port failed. Delivered once; the state is Lost from now on.
};

// True for the bit rates CANopen and slcan have a code for: 10000, 20000, 50000, 100000, 125000, 250000, 500000,
// 800000 and 1000000 bit/s.
bool validBitrate(unsigned bitrate) noexcept;

struct LinkConfig {
    // "COM3" or "/dev/ttyACM0" selects an slcan serial adapter. Anything else
    // (can0, vcan0, slcan0) is a SocketCAN interface (Linux only).
    const char* interface = "";
    // Bit rate in bit/s: 10000, 20000, 50000, 100000, 125000, 250000, 500000,
    // 800000 or 1000000. A serial adapter is told this rate. A SocketCAN
    // interface is configured by the OS (`ip link`), so the value is only checked.
    unsigned bitrate = 500000;
    // Speed of the serial line itself, for real UARTs only.
    unsigned serialBaud = 115200;
};

class Link {
public:
    Link() noexcept = default;
    ~Link();
    Link(const Link&) = delete;
    Link& operator=(const Link&) = delete;

    // Opens the port. On false, lastError() says why, and the state is
    // Closed. An open link is closed first.
    bool open(const LinkConfig& config) noexcept;

    // Closes the port. Always ends Closed. False if the OS reported an error
    // while closing (lastError().code == CloseFailed): nothing to retry,
    // the handle is gone, so report it and move on. Closing a link that is
    // not open succeeds.
    bool close() noexcept;

    LinkState state() const noexcept { return state_; }

    // The error of the last call that failed, or of the loss. Stays until the
    // next open() or close().
    LinkError lastError() const noexcept { return error_; }

    // Non-blocking. False if the link is not open or the queue is full.
    bool send(const CanFrame& frame) noexcept;

    // Waits up to `timeout` for a frame. Call it only while the state is
    // Open: in any other state it returns Idle at once, without waiting.
    // Returns Idle early when wake() is called.
    // On Lost the port has been released, and lastError() holds the reason.
    LinkEvent poll(CanFrame& out, Duration timeout) noexcept;

    // Makes a poll() that is waiting, or the next one, return Idle early.
    // Any thread, any time (also while the link is closed). A wake is never
    // lost, so after open() a loop checks its flags before its first poll(),
    // and a wake that came before it just makes that poll() return at once.
    void wake() noexcept { waker_.wake(); }

    // For the I/O thread when the link is not open and there is nothing to
    // read: waits up to `timeout`, and returns early (true) when wake() is
    // called. The same wake-up as poll() uses, so the thread has one way to
    // be interrupted whether it is connected or not.
    bool idle(Duration timeout) noexcept { return waker_.wait(timeout); }

    // The CAN controller state, for the master's own error handling.
    bool busStatusChanged() const noexcept;
    BusStatus takeBusStatus() noexcept;

    // The port as an ICanBus, for a Master. Valid only while the state is Open.
    ICanBus* bus() noexcept { return active_; }

    // Writes a text for the error ("code: OS message") and returns its length.
    // Truncated if it does not fit; always terminated when capacity > 0.
    static std::size_t describe(const LinkError& error, char* buffer, std::size_t capacity) noexcept;

private:
    int portOsError() const noexcept;
    bool closePort() noexcept;
    void setError(LinkErrc code, int osError) noexcept { error_ = { code, osError }; }

    slcan::SlcanBus slcan_;
#if defined(__linux__)
    linux_port::SocketCanBus socketCan_;
#endif
    Waker waker_;
    ICanBus* active_ = nullptr;
    LinkState state_ = LinkState::Closed;
    LinkError error_;
    bool serial_ = false;  // which of the two ports is active
};

} // namespace canopen_conductor
