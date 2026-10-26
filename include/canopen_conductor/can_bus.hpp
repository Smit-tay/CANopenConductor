#pragma once

#include "canopen_conductor/frame.hpp"
#include "canopen_conductor/time.hpp"

namespace canopen_conductor {

enum class RxStatus {
    Frame,    // a frame was received
    Timeout,  // nothing arrived in time
    Error,    // interface lost or unusable
};

// CAN controller state as seen by the host port.
enum class BusState : std::uint8_t {
    Active,   // error active (or warning)
    Passive,  // error passive
    BusOff,
};

// Reported by the host port; a Device turns it into EMCY and error behaviour.
struct BusStatus {
    BusState state = BusState::Active;
    bool rxOverflow = false;  // frames were lost on receive since the last report
    bool txOverflow = false;  // frames were lost on transmit since the last report
};

// The only path to the CAN hardware. Implemented by the host port.
class ICanBus {
public:
    // Non-blocking. Queue the frame and return true, or return false if the
    // send queue is full. Must never wait.
    virtual bool send(const CanFrame& frame) noexcept = 0;

    // Called by the host I/O loop only; the library never reads the bus.
    virtual RxStatus receive(CanFrame& out, Duration timeout) noexcept = 0;

protected:
    ICanBus() = default;
    ~ICanBus() = default;
    ICanBus(const ICanBus&) = default;
    ICanBus& operator=(const ICanBus&) = default;
};

} // namespace canopen_conductor
