#include "canopen_conductor/pdo.hpp"

#include <cstring>

namespace canopen_conductor {

namespace {

bool isSigned(DataType t) noexcept {
    switch (t) {
    case DataType::Integer8:
    case DataType::Integer16:
    case DataType::Integer24:
    case DataType::Integer32:
    case DataType::Integer40:
    case DataType::Integer48:
    case DataType::Integer56:
    case DataType::Integer64:
        return true;
    default:
        return false;
    }
}

// CiA 301 7.4.7.1: data type indices that may be mapped as dummies.
bool isDummyIndex(std::uint16_t index) noexcept {
    return (index >= 0x0001 && index <= 0x0007) || index == 0x0010 || (index >= 0x0012 && index <= 0x0016) ||
           (index >= 0x0018 && index <= 0x001B);
}

} // namespace

unsigned pdoBits(DataType type) noexcept {
    switch (type) {
    case DataType::Boolean:
        return 1;
    case DataType::Integer8:
    case DataType::Unsigned8:
        return 8;
    case DataType::Integer16:
    case DataType::Unsigned16:
        return 16;
    case DataType::Integer24:
    case DataType::Unsigned24:
        return 24;
    case DataType::Integer32:
    case DataType::Unsigned32:
    case DataType::Real32:
        return 32;
    case DataType::Integer40:
    case DataType::Unsigned40:
        return 40;
    case DataType::Integer48:
    case DataType::Unsigned48:
        return 48;
    case DataType::Integer56:
    case DataType::Unsigned56:
        return 56;
    case DataType::Integer64:
    case DataType::Unsigned64:
    case DataType::Real64:
        return 64;
    default:
        return 0;
    }
}

bool validPdoEntries(const PdoEntry* entries, std::size_t count) noexcept {
    if (count > 64 || (count > 0 && entries == nullptr)) {
        return false;
    }
    unsigned total = 0;
    for (std::size_t k = 0; k < count; ++k) {
        const PdoEntry& e = entries[k];
        const unsigned bits = pdoBits(e.type);
        if (bits == 0) {
            return false;
        }
        if (e.index <= 0x001F && (!isDummyIndex(e.index) || static_cast<std::uint16_t>(e.type) != e.index)) {
            return false;  // index 0000h..001Fh is only allowed as a well-formed dummy
        }
        total += bits;
    }
    return total <= 64;
}

bool validPdoCobId(std::uint32_t id) noexcept {
    if (id > 0x7FF) {
        return false;
    }
    const bool restricted = id <= 0x07F || (id >= 0x101 && id <= 0x180) || (id >= 0x581 && id <= 0x5FF) ||
                            (id >= 0x601 && id <= 0x67F) || (id >= 0x6E0 && id <= 0x6FF) ||
                            (id >= 0x701 && id <= 0x77F) || id >= 0x780;
    const bool syncEmcyTime = id >= 0x080 && id <= 0x100;
    return !restricted && !syncEmcyTime;
}

int decodePdo(const PdoEntry* entries, std::size_t count, const CanFrame& frame, PdoValue* out) noexcept {
    unsigned total = 0;
    for (std::size_t k = 0; k < count; ++k) {
        total += pdoBits(entries[k].type);
    }
    if (frame.len < (total + 7) / 8) {
        return -1;
    }

    // Frame bytes as one little-endian bit string.
    std::uint64_t raw = 0;
    const std::uint8_t len = frame.len > 8 ? 8 : frame.len;
    for (std::uint8_t b = 0; b < len; ++b) {
        raw |= static_cast<std::uint64_t>(frame.data[b]) << (8 * b);
    }

    int n = 0;
    unsigned offset = 0;
    for (std::size_t k = 0; k < count; ++k) {
        const PdoEntry& e = entries[k];
        const unsigned bits = pdoBits(e.type);
        const std::uint64_t mask = bits == 64 ? ~std::uint64_t{0} : (std::uint64_t{1} << bits) - 1;
        const std::uint64_t v = (raw >> offset) & mask;
        offset += bits;
        if (e.index <= 0x001F) {
            continue;  // dummy
        }

        PdoValue& o = out[n++];
        o.index = e.index;
        o.subIndex = e.subIndex;
        o.type = e.type;
        o.u = v;
        o.i = static_cast<std::int64_t>(v);
        if (isSigned(e.type) && bits < 64 && ((v >> (bits - 1)) & 1) != 0) {
            o.i = static_cast<std::int64_t>(v | ~mask);  // sign-extend
        }
        if (e.type == DataType::Real32) {
            const auto b32 = static_cast<std::uint32_t>(v);
            float x;
            std::memcpy(&x, &b32, sizeof x);
            o.f = static_cast<double>(x);
        } else if (e.type == DataType::Real64) {
            double x;
            std::memcpy(&x, &v, sizeof x);
            o.f = x;
        } else if (isSigned(e.type)) {
            o.f = static_cast<double>(o.i);
        } else {
            o.f = static_cast<double>(v);
        }
    }
    return n;
}

} // namespace canopen_conductor
