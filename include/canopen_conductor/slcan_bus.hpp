#pragma once

// slcan host port: an ICanBus over a serial-line CAN adapter (Linux and
// Windows). Host layer: this talks to the operating system.

#include "canopen_conductor/can_bus.hpp"
#include "canopen_conductor/serial_port.hpp"
#include "canopen_conductor/slcan_codec.hpp"

#include <cstddef>

namespace canopen_conductor::slcan {

class SlcanBus final : public ICanBus {
public:
    // Opens `device` ("/dev/ttyACM0", "COM3") and starts the CAN channel at
    // `canKbit` (10, 20, 50, 100, 125, 250, 500, 800, 1000).
    bool open(const char* device, unsigned serialBaud, unsigned canKbit) noexcept;
    // False if the OS reported an error while closing. The port is closed in any case.
    bool close() noexcept;

    // Makes a blocked receive() return early (RxStatus::Timeout). Not owned; set before open().
    void setWaker(Waker* waker) noexcept { port_.setWaker(waker); }
    // The OS error behind the last failed open(), send() or receive(); 0 if none.
    int osError() const noexcept { return port_.osError(); }

    bool send(const CanFrame& frame) noexcept override;
    RxStatus receive(CanFrame& out, Duration timeout) noexcept override;

    // Ask the adapter for its status flags (reply parsed by receive()).
    bool pollStatus() noexcept;
    bool busStatusChanged() const noexcept { return changed_; }
    BusStatus takeBusStatus() noexcept;

private:
    bool queue(const char* text, std::size_t n) noexcept;
    bool flush() noexcept;
    void onLine(const char* line, std::size_t n) noexcept;

    SerialPort port_;
    LineAssembler lines_;
    bool lost_ = false;

    char tx_[4096] = {};
    std::size_t txHead_ = 0;
    std::size_t txCount_ = 0;

    CanFrame rx_[64] = {};
    std::size_t rxHead_ = 0;
    std::size_t rxCount_ = 0;

    BusStatus status_{};
    bool changed_ = false;
};

} // namespace canopen_conductor::slcan
