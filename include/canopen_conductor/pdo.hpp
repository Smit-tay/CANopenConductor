#pragma once

#include "canopen_conductor/frame.hpp"
#include "canopen_conductor/time.hpp"

#include <cstddef>
#include <cstdint>

namespace canopen_conductor {

// Object dictionary data type indices (CiA 301 table 44).
enum class DataType : std::uint16_t {
    Boolean = 0x0001,
    Integer8 = 0x0002,
    Integer16 = 0x0003,
    Integer32 = 0x0004,
    Unsigned8 = 0x0005,
    Unsigned16 = 0x0006,
    Unsigned32 = 0x0007,
    Real32 = 0x0008,
    VisibleString = 0x0009,
    OctetString = 0x000A,
    UnicodeString = 0x000B,
    TimeOfDay = 0x000C,
    TimeDifference = 0x000D,
    Domain = 0x000F,
    Integer24 = 0x0010,
    Real64 = 0x0011,
    Integer40 = 0x0012,
    Integer48 = 0x0013,
    Integer56 = 0x0014,
    Integer64 = 0x0015,
    Unsigned24 = 0x0016,
    Unsigned40 = 0x0018,
    Unsigned48 = 0x0019,
    Unsigned56 = 0x001A,
    Unsigned64 = 0x001B,
};

// Size in bits of a type that can be decoded from a PDO; 0 for any other.
unsigned pdoBits(DataType type) noexcept;

// One mapped object, in mapping order (as in 1A00h sub 1..n). An entry
// whose index is a data type index is a dummy: it takes up bits, no value.
struct PdoEntry {
    std::uint16_t index = 0;
    std::uint8_t subIndex = 0;
    DataType type = DataType::Unsigned8;
};

// One slave TPDO.
struct PdoConfig {
    std::uint8_t nodeId = 0;            // must be a configured node
    std::uint16_t cobId = 0;            // 11-bit
    const PdoEntry* entries = nullptr;  // copied by Master::create / addPdo
    std::uint8_t entryCount = 0;        // 0..64; 0 for an MPDO
    Duration timeout{};                 // deadline monitoring; 0 = off
    bool mpdo = false;                  // multiplexed PDO, reported through MpdoFn
};

// One decoded value.
struct PdoValue {
    std::uint16_t index = 0;
    std::uint8_t subIndex = 0;
    DataType type = DataType::Unsigned8;
    std::uint64_t u = 0;  // raw bits, zero-extended
    std::int64_t i = 0;   // integer value (signed types sign-extended); raw bits for REAL types
    double f = 0.0;       // value as a double (REAL32/REAL64 decoded, integers converted)
};

struct PdoEvent {
    std::uint8_t nodeId = 0;
    std::uint16_t cobId = 0;
    bool lengthError = false;          // frame shorter than the mapping; no values
    bool timeout = false;              // deadline missed; no values
    const PdoValue* values = nullptr;  // valid only during the callback
    std::uint8_t count = 0;
    std::uint32_t syncCount = 0;       // SYNCs sent or seen by the master so far
};

// Called on the I/O thread. Must only signal.
using PdoFn = void (*)(void* context, const PdoEvent& event) noexcept;

// One received multiplexed PDO (CiA 301 7.2.3.4).
struct MpdoEvent {
    std::uint8_t nodeId = 0;      // producer, from the PdoConfig
    std::uint16_t cobId = 0;
    bool destination = false;     // true: DAM (address = consumer), false: SAM (address = producer)
    std::uint8_t address = 0;     // node-id in byte 0; 0 in DAM = all consumers
    std::uint16_t index = 0;
    std::uint8_t subIndex = 0;
    std::uint8_t data[4] = {};
    std::uint32_t syncCount = 0;
};

// Called on the I/O thread. Must only signal.
using MpdoFn = void (*)(void* context, const MpdoEvent& event) noexcept;

// Entries use supported types, dummies are well formed, at most 64 entries
// and 64 bits in total.
bool validPdoEntries(const PdoEntry* entries, std::size_t count) noexcept;

// 11-bit, not restricted (CiA 301 table 40), not in 080h..100h.
bool validPdoCobId(std::uint32_t cobId) noexcept;

// Decode `frame` with valid `entries` into `out` (room for `count` values).
// Returns the number of values written, or -1 if the frame is too short.
int decodePdo(const PdoEntry* entries, std::size_t count, const CanFrame& frame, PdoValue* out) noexcept;

} // namespace canopen_conductor
