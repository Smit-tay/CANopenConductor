#pragma once

#include <cstdint>
#include <type_traits>

namespace canopen_conductor {

// One classic CAN frame. CANopen uses 11-bit base frames (CiA 301 6.2).
struct CanFrame {
    std::uint32_t id = 0;       // CAN identifier
    std::uint8_t len = 0;       // data length, 0..8
    bool rtr = false;           // remote transmission request
    bool extended = false;      // 29-bit identifier
    std::uint8_t data[8] = {};
};

static_assert(std::is_trivially_copyable_v<CanFrame>);

// Equal if header and the first `len` data bytes match.
constexpr bool operator==(const CanFrame& a, const CanFrame& b) noexcept {
    if (a.id != b.id || a.len != b.len || a.rtr != b.rtr || a.extended != b.extended) {
        return false;
    }
    for (std::uint8_t i = 0; i < a.len && i < 8; ++i) {
        if (a.data[i] != b.data[i]) {
            return false;
        }
    }
    return true;
}

// True for frames the library may process: 11-bit data frames of 0..8 bytes.
constexpr bool isUsable(const CanFrame& f) noexcept {
    return !f.rtr && !f.extended && f.len <= 8 && f.id <= 0x7FF;
}

} // namespace canopen_conductor
