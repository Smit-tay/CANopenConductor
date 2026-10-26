#pragma once

#include "canopen_conductor/frame.hpp"

#include <cstdint>

namespace canopen_conductor {

// One received emergency message (CiA 301 7.2.7.3).
struct EmcyEvent {
    std::uint8_t nodeId = 0;
    std::uint16_t errorCode = 0;       // 0000h = error reset / no error
    std::uint8_t errorRegister = 0;    // 1001h of the sender
    std::uint8_t manufacturer[5] = {};
};

// Called on the I/O thread. Must only signal.
using EmcyFn = void (*)(void* context, const EmcyEvent& event) noexcept;

// True if `f` is an EMCY frame (0x081..0x0FF, 8 bytes); fills `out`.
constexpr bool decodeEmcy(const CanFrame& f, EmcyEvent& out) noexcept {
    if (!isUsable(f) || f.len != 8 || f.id < 0x081 || f.id > 0x0FF) {
        return false;
    }
    out.nodeId = static_cast<std::uint8_t>(f.id - 0x080);
    out.errorCode = static_cast<std::uint16_t>(f.data[0] | (f.data[1] << 8));
    out.errorRegister = f.data[2];
    for (int i = 0; i < 5; ++i) {
        out.manufacturer[i] = f.data[3 + i];
    }
    return true;
}

} // namespace canopen_conductor
