#include "slcan_codec.hpp"

namespace canopen_conductor::slcan {

namespace {

constexpr char kHex[] = "0123456789ABCDEF";

int hexValue(char c) noexcept {
    if (c >= '0' && c <= '9') {
        return c - '0';
    }
    if (c >= 'A' && c <= 'F') {
        return c - 'A' + 10;
    }
    if (c >= 'a' && c <= 'f') {
        return c - 'a' + 10;
    }
    return -1;
}

// Read `digits` hex digits at p; false on a non-hex character.
bool readHex(const char* p, std::size_t digits, std::uint32_t& out) noexcept {
    out = 0;
    for (std::size_t i = 0; i < digits; ++i) {
        const int v = hexValue(p[i]);
        if (v < 0) {
            return false;
        }
        out = (out << 4) | static_cast<std::uint32_t>(v);
    }
    return true;
}

} // namespace

std::size_t encode(const CanFrame& f, char* out) noexcept {
    if (f.len > 8 || (!f.extended && f.id > 0x7FF) || (f.extended && f.id > 0x1FFFFFFF)) {
        return 0;
    }
    std::size_t n = 0;
    out[n++] = f.extended ? (f.rtr ? 'R' : 'T') : (f.rtr ? 'r' : 't');
    const int idDigits = f.extended ? 8 : 3;
    for (int i = idDigits - 1; i >= 0; --i) {
        out[n++] = kHex[(f.id >> (4 * i)) & 0xF];
    }
    out[n++] = kHex[f.len];
    if (!f.rtr) {
        for (std::uint8_t i = 0; i < f.len; ++i) {
            out[n++] = kHex[f.data[i] >> 4];
            out[n++] = kHex[f.data[i] & 0xF];
        }
    }
    out[n++] = '\r';
    return n;
}

bool decode(const char* line, std::size_t length, CanFrame& out) noexcept {
    if (length < 1) {
        return false;
    }
    const char kind = line[0];
    if (kind != 't' && kind != 'T' && kind != 'r' && kind != 'R') {
        return false;
    }
    CanFrame f;
    f.extended = kind == 'T' || kind == 'R';
    f.rtr = kind == 'r' || kind == 'R';
    const std::size_t idDigits = f.extended ? 8 : 3;
    std::uint32_t v = 0;
    if (length < 1 + idDigits + 1 || !readHex(line + 1, idDigits, v)) {
        return false;
    }
    f.id = v;
    std::uint32_t len = 0;
    if (!readHex(line + 1 + idDigits, 1, len) || len > 8) {
        return false;
    }
    f.len = static_cast<std::uint8_t>(len);
    const std::size_t dataStart = 2 + idDigits;
    const std::size_t dataDigits = f.rtr ? 0 : 2u * f.len;
    // Exactly the data, or the data plus a 4-digit time stamp.
    if (length != dataStart + dataDigits && length != dataStart + dataDigits + 4) {
        return false;
    }
    for (std::size_t i = 0; i < f.len && !f.rtr; ++i) {
        if (!readHex(line + dataStart + 2 * i, 2, v)) {
            return false;
        }
        f.data[i] = static_cast<std::uint8_t>(v);
    }
    if ((!f.extended && f.id > 0x7FF) || f.id > 0x1FFFFFFF) {
        return false;
    }
    out = f;
    return true;
}

bool decodeStatus(const char* line, std::size_t length, BusStatus& out) noexcept {
    std::uint32_t flags = 0;
    if (length != 3 || line[0] != 'F' || !readHex(line + 1, 2, flags)) {
        return false;
    }
    out.state = (flags & 0x20) != 0 ? BusState::Passive : BusState::Active;
    out.rxOverflow = (flags & 0x09) != 0;
    out.txOverflow = (flags & 0x02) != 0;
    return true;
}

const char* bitrateCommand(unsigned kbit) noexcept {
    switch (kbit) {
    case 10:
        return "S0";
    case 20:
        return "S1";
    case 50:
        return "S2";
    case 100:
        return "S3";
    case 125:
        return "S4";
    case 250:
        return "S5";
    case 500:
        return "S6";
    case 800:
        return "S7";
    case 1000:
        return "S8";
    default:
        return nullptr;
    }
}

bool LineAssembler::push(char c) noexcept {
    if (c == '\r' || c == '\a') {
        const bool ok = !overflow_;
        len_ = fill_;
        buf_[len_] = '\0';
        fill_ = 0;
        overflow_ = false;
        return ok;  // an empty line is a plain OK reply
    }
    if (c == '\n') {
        return false;  // some adapters add LF; ignore it
    }
    if (fill_ >= kMaxLine) {
        overflow_ = true;  // garbage: drop the whole line
        return false;
    }
    buf_[fill_++] = c;
    return false;
}

} // namespace canopen_conductor::slcan
