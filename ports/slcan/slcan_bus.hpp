#pragma once

// slcan host port: an ICanBus over a serial-line CAN adapter (Linux and
// Windows). Not part of the library.

#include "canopen_conductor/can_bus.hpp"
#include "serial_port.hpp"
#include "slcan_codec.hpp"

#include <cstddef>

namespace canopen_conductor::slcan {

class SlcanBus final : public ICanBus {
public:
    // Opens `device` ("/dev/ttyACM0", "COM3") and starts the CAN channel at
    // `canKbit` (10, 20, 50, 100, 125, 250, 500, 800, 1000).
    bool open(const char* device, unsigned serialBaud, unsigned canKbit) noexcept;
    void close() noexcept;

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
