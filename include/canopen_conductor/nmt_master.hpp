#pragma once

#include "canopen_conductor/can_bus.hpp"

#include <cstdint>

namespace canopen_conductor {

// NMT node control command specifiers (CiA 301 7.2.8.3.1).
enum class NmtCommand : std::uint8_t {
    Start = 0x01,
    Stop = 0x02,
    EnterPreOperational = 0x80,
    ResetNode = 0x81,
    ResetCommunication = 0x82,
};

struct NmtMasterConfig {
    // Own heartbeat producer. Off when node-id or period is 0.
    std::uint8_t heartbeatNodeId = 0;
    Duration heartbeatPeriod{};
    std::uint8_t heartbeatState = 0x05;  // operational
};

// Sends NMT commands and, optionally, the master's own heartbeat.
class NmtMaster {
public:
    NmtMaster(ICanBus& bus, const NmtMasterConfig& config) noexcept;

    // Send `command` to `nodeId` (0 = all nodes). True if sent or queued for
    // the next tick; false if nodeId/command is invalid or an earlier
    // command is still waiting for room in the send queue.
    bool command(NmtCommand command, std::uint8_t nodeId) noexcept;

    void tick(TimePoint now) noexcept;

    // Earliest time tick() has work to do if no frame arrives first:
    // `now` if a frame is waiting for room in the send queue, a past time if
    // something is already due, TimePoint::max() if nothing is timed.
    TimePoint nextDeadline(TimePoint now) const noexcept;

    bool commandPending() const noexcept { return pending_; }

    // State byte of the own heartbeat from now on.
    void setHeartbeatState(std::uint8_t state) noexcept { config_.heartbeatState = state; }

private:
    bool producerOn() const noexcept;

    ICanBus* bus_;
    NmtMasterConfig config_;
    CanFrame commandFrame_{};
    bool pending_ = false;
    bool heartbeatStarted_ = false;
    TimePoint heartbeatDue_{};
};

} // namespace canopen_conductor
