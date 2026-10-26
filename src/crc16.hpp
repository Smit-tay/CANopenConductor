#pragma once

// CRC used by SDO block transfer (CiA 301 7.2.4.3.16):
// polynomial x^16 + x^12 + x^5 + 1 (0x1021), start value 0, MSB first.

#include <array>
#include <cstddef>
#include <cstdint>

namespace canopen_conductor::detail {

inline constexpr std::array<std::uint16_t, 256> crc16Table = [] {
    std::array<std::uint16_t, 256> table{};
    for (unsigned byte = 0; byte < 256; ++byte) {
        unsigned value = byte << 8;
        for (int bit = 0; bit < 8; ++bit) {
            value = (value & 0x8000u) ? ((value << 1) ^ 0x1021u) : (value << 1);
        }
        table[byte] = static_cast<std::uint16_t>(value);
    }
    return table;
}();

// Continue a CRC over `count` more bytes.
constexpr std::uint16_t crc16(std::uint16_t crc, const std::uint8_t* data, std::size_t count) noexcept {
    for (std::size_t i = 0; i < count; ++i) {
        const unsigned top = static_cast<unsigned>(crc >> 8) ^ data[i];
        crc = static_cast<std::uint16_t>((crc << 8) ^ crc16Table[top]);
    }
    return crc;
}

} // namespace canopen_conductor::detail
