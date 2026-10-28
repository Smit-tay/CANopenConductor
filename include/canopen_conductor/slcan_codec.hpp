#pragma once

// slcan (Lawicel ASCII) encoding and decoding. Portable, no I/O.

#include "canopen_conductor/can_bus.hpp"

#include <cstddef>

namespace canopen_conductor::slcan {

inline constexpr std::size_t kMaxLine = 40;  // longest line we accept (frame + time stamp)

// Encode a frame as "t/T/r/R..." plus CR into `out` (room for kMaxLine).
// Returns the number of characters, 0 if the frame is invalid.
std::size_t encode(const CanFrame& f, char* out) noexcept;

// Decode one line (without the CR). False if it is not a frame line.
bool decode(const char* line, std::size_t length, CanFrame& out) noexcept;

// Decode a status reply "Fxx". False if it is not one.
bool decodeStatus(const char* line, std::size_t length, BusStatus& out) noexcept;

// "Sn" for a CAN bit rate in kbit/s, or nullptr if slcan has no code for it.
const char* bitrateCommand(unsigned kbit) noexcept;

// Collects bytes into lines ended by CR. BEL (error reply) also ends a line.
class LineAssembler {
public:
    // Add one byte; true when a complete line is ready in line()/length().
    bool push(char c) noexcept;
    const char* line() const noexcept { return buf_; }
    std::size_t length() const noexcept { return len_; }

private:
    char buf_[kMaxLine + 1] = {};
    std::size_t len_ = 0;
    std::size_t fill_ = 0;
    bool overflow_ = false;
};

} // namespace canopen_conductor::slcan
