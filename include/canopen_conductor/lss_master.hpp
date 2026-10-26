#pragma once

#include "canopen_conductor/can_bus.hpp"

#include <cstdint>

namespace canopen_conductor {

// LSS address: identity object 1018h subs 1..4.
struct LssAddress {
    std::uint32_t vendorId = 0;
    std::uint32_t productCode = 0;
    std::uint32_t revision = 0;
    std::uint32_t serial = 0;
};

enum class LssOp : std::uint8_t {
    SwitchSelective,
    ConfigureNodeId,
    ConfigureBitTiming,
    Store,
    Inquire,
    Fastscan,
};

enum class LssStatus : std::uint8_t {
    Ok,
    Timeout,   // no matching response
    Rejected,  // configure/store answered with an error; see errorCode
    NoNode,    // fastscan: no unconfigured slave answered
    NoAck,     // fastscan: a verify step was not confirmed
};

struct LssResult {
    LssOp op = LssOp::Inquire;
    LssStatus status = LssStatus::Ok;
    std::uint8_t errorCode = 0;          // configure/store: slave error code
    std::uint8_t manufacturerError = 0;  // when errorCode == FFh
    std::uint32_t value = 0;             // inquire
    LssAddress address{};                // fastscan: found address
    std::uint8_t knownParts = 0;         // fastscan: bit 0 vendor .. bit 3 serial
};

// Called on the I/O thread. Must only signal.
using LssDoneFn = void (*)(void* context, const LssResult& result) noexcept;

enum class LssInquire : std::uint8_t {
    Vendor = 0x5A,
    Product = 0x5B,
    Revision = 0x5C,
    Serial = 0x5D,
    NodeId = 0x5E,
};

enum class LssScan : std::uint8_t {
    Scan,   // find the value
    Match,  // value given in LssFastscan::match, only verified
    Skip,   // not used (vendor cannot be skipped; at most two parts)
};

struct LssFastscan {
    LssScan vendor = LssScan::Scan;
    LssScan product = LssScan::Scan;
    LssScan revision = LssScan::Scan;
    LssScan serial = LssScan::Scan;
    LssAddress match{};
};

enum class LssStart {
    Started,          // confirmed service running, or unconfirmed one sent/queued
    Busy,             // another LSS operation is running
    WrongState,       // not allowed in the current selection state
    InvalidArgument,
};

struct LssConfig {
    Duration timeout = Duration{1'000'000};        // confirmed services
    Duration fastscanWindow = Duration{100'000};   // per fastscan step
    LssDoneFn onDone = nullptr;
    void* context = nullptr;
};

// LSS master (commissioning). CAN-IDs 7E5h (requests) and 7E4h (responses).
class LssMaster {
public:
    LssMaster(ICanBus& bus, const LssConfig& config) noexcept;

    // Unconfirmed services: no callback.
    LssStart switchGlobal(bool configuration, TimePoint now) noexcept;
    LssStart activateBitTiming(std::uint16_t delayMs, TimePoint now) noexcept;

    // Confirmed services: result through the callback.
    LssStart switchSelective(const LssAddress& address, TimePoint now) noexcept;
    LssStart configureNodeId(std::uint8_t nodeId, TimePoint now) noexcept;   // 1..127 or FFh
    LssStart configureBitTiming(std::uint8_t tableIndex, TimePoint now) noexcept;
    LssStart store(TimePoint now) noexcept;
    LssStart inquire(LssInquire what, TimePoint now) noexcept;
    LssStart fastscan(const LssFastscan& request, TimePoint now) noexcept;

    void onFrame(const CanFrame& frame, TimePoint now) noexcept;
    void tick(TimePoint now) noexcept;

    // Earliest time tick() has work to do if no frame arrives first:
    // `now` if a frame is waiting for room in the send queue, a past time if
    // something is already due, TimePoint::max() if nothing is timed.
    TimePoint nextDeadline(TimePoint now) const noexcept;

    bool busy() const noexcept { return phase_ != Phase::Idle; }

private:
    enum class Selection : std::uint8_t { None, Global, Selective };
    enum class Phase : std::uint8_t { Idle, Response, FastscanCheck, FastscanScan, FastscanVerify };

    LssStart canStart(Selection required1, Selection required2, unsigned frames) const noexcept;
    void queue(std::uint8_t command, std::uint32_t value, std::uint8_t b5 = 0, std::uint8_t b6 = 0,
               std::uint8_t b7 = 0) noexcept;
    void startConfirmed(LssOp op, std::uint8_t expect, TimePoint now) noexcept;
    void pump(TimePoint now) noexcept;
    void finish(LssStatus status) noexcept;

    LssScan mode(std::uint8_t part) const noexcept;
    std::uint8_t nextPart(std::uint8_t part) const noexcept;
    void sendFastscan(std::uint32_t id, std::uint8_t bit, std::uint8_t sub, std::uint8_t next,
                      TimePoint now) noexcept;
    void beginPart(TimePoint now) noexcept;
    void beginVerify(TimePoint now) noexcept;
    void fastscanStepDone(TimePoint now) noexcept;

    ICanBus* bus_;
    LssConfig config_;
    Selection selection_ = Selection::None;
    Phase phase_ = Phase::Idle;

    // Frames waiting for room in the send queue.
    CanFrame queue_[4]{};
    std::uint8_t queueHead_ = 0;
    std::uint8_t queueCount_ = 0;

    // Timer: runs once every queued frame of the current step is on the bus.
    bool timerRunning_ = false;
    TimePoint timerStart_{};

    LssResult result_{};
    std::uint8_t expect_ = 0;  // command byte of the awaited response

    // Fastscan.
    LssFastscan fs_{};
    std::uint8_t part_ = 0;
    std::uint8_t bit_ = 0;
    std::uint32_t id_ = 0;
    bool acked_ = false;
};

} // namespace canopen_conductor
