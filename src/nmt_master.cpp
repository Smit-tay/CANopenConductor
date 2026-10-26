#include "canopen_conductor/nmt_master.hpp"

namespace canopen_conductor {

namespace {

bool knownCommand(NmtCommand c) noexcept {
    switch (c) {
    case NmtCommand::Start:
    case NmtCommand::Stop:
    case NmtCommand::EnterPreOperational:
    case NmtCommand::ResetNode:
    case NmtCommand::ResetCommunication:
        return true;
    }
    return false;
}

} // namespace

NmtMaster::NmtMaster(ICanBus& bus, const NmtMasterConfig& config) noexcept : bus_(&bus), config_(config) {}

bool NmtMaster::command(NmtCommand command, std::uint8_t nodeId) noexcept {
    if (pending_ || nodeId > 127 || !knownCommand(command)) {
        return false;
    }
    commandFrame_ = CanFrame{};
    commandFrame_.id = 0x000;
    commandFrame_.len = 2;
    commandFrame_.data[0] = static_cast<std::uint8_t>(command);
    commandFrame_.data[1] = nodeId;
    pending_ = !bus_->send(commandFrame_);
    return true;
}

bool NmtMaster::producerOn() const noexcept {
    return config_.heartbeatNodeId >= 1 && config_.heartbeatNodeId <= 127 &&
           config_.heartbeatPeriod > Duration::zero();
}

TimePoint NmtMaster::nextDeadline(TimePoint now) const noexcept {
    if (pending_) {
        return now;
    }
    if (!producerOn()) {
        return TimePoint::max();
    }
    return heartbeatStarted_ ? heartbeatDue_ : now;
}

void NmtMaster::tick(TimePoint now) noexcept {
    if (pending_ && bus_->send(commandFrame_)) {
        pending_ = false;
    }

    if (!producerOn()) {
        return;
    }
    if (!heartbeatStarted_) {
        heartbeatStarted_ = true;
        heartbeatDue_ = now;
    }
    if (now < heartbeatDue_) {
        return;
    }
    CanFrame f;
    f.id = 0x700u + config_.heartbeatNodeId;
    f.len = 1;
    f.data[0] = config_.heartbeatState;
    if (!bus_->send(f)) {
        return;  // still due; retried next tick
    }
    heartbeatDue_ += config_.heartbeatPeriod;
    if (heartbeatDue_ <= now) {
        heartbeatDue_ = now + config_.heartbeatPeriod;  // fell behind: no catch-up burst
    }
}

} // namespace canopen_conductor
