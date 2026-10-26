#include "canopen_conductor/node_table.hpp"

namespace canopen_conductor {

namespace {

bool knownState(std::uint8_t value, NmtState& out) noexcept {
    switch (value) {
    case 4:
        out = NmtState::Stopped;
        return true;
    case 5:
        out = NmtState::Operational;
        return true;
    case 127:
        out = NmtState::PreOperational;
        return true;
    default:
        return false;
    }
}

} // namespace

bool NodeTable::valid(const NodeConfig& c) noexcept {
    const bool guarding = c.guardTime > Duration::zero() && c.lifeTimeFactor > 0;
    return c.nodeId >= 1 && c.nodeId <= 127 && c.heartbeatTimeout >= Duration::zero() &&
           c.guardTime >= Duration::zero() && !(guarding && c.heartbeatTimeout > Duration::zero());
}

bool NodeTable::valid(const NodeConfig* configs, std::size_t count) noexcept {
    if (count > 127 || (count > 0 && configs == nullptr)) {
        return false;
    }
    bool seen[128] = {};
    for (std::size_t i = 0; i < count; ++i) {
        const NodeConfig& c = configs[i];
        if (!valid(c) || seen[c.nodeId]) {
            return false;
        }
        seen[c.nodeId] = true;
    }
    return true;
}

bool NodeTable::init(NodeStatus* storage, std::size_t capacity, const NodeConfig* configs, std::size_t count,
                     NodeEventFn onEvent, void* context, ICanBus* bus) noexcept {
    nodes_ = nullptr;
    count_ = 0;
    capacity_ = 0;
    for (auto& s : slot_) {
        s = kNoSlot;
    }
    if (!valid(configs, count) || count > capacity || capacity > 127 || (capacity > 0 && storage == nullptr)) {
        return false;
    }
    nodes_ = storage;
    capacity_ = capacity;
    onEvent_ = onEvent;
    context_ = context;
    bus_ = bus;
    for (std::size_t i = 0; i < count; ++i) {
        add(configs[i]);
    }
    return true;
}

bool NodeTable::add(const NodeConfig& c) noexcept {
    if (count_ >= capacity_ || !valid(c) || slot_[c.nodeId] != kNoSlot) {
        return false;
    }
    NodeStatus& s = nodes_[count_];
    s = NodeStatus{};
    s.nodeId = c.nodeId;
    s.heartbeatTimeout = c.heartbeatTimeout;
    s.guardTime = c.guardTime;
    s.lifeTimeFactor = c.lifeTimeFactor;
    slot_[c.nodeId] = static_cast<std::uint8_t>(count_);
    ++count_;
    return true;
}

std::size_t NodeTable::remove(std::uint8_t nodeId) noexcept {
    const std::size_t i = indexOf(nodeId);
    if (i >= count_) {
        return count_;
    }
    slot_[nodeId] = kNoSlot;
    const std::size_t last = count_ - 1;
    if (i != last) {
        nodes_[i] = nodes_[last];
        slot_[nodes_[i].nodeId] = static_cast<std::uint8_t>(i);
    }
    --count_;
    return i;
}

std::size_t NodeTable::indexOf(std::uint8_t nodeId) const noexcept {
    if (nodeId > 127 || slot_[nodeId] == kNoSlot) {
        return count_;
    }
    return slot_[nodeId];
}

const NodeStatus* NodeTable::find(std::uint8_t nodeId) const noexcept {
    const std::size_t i = indexOf(nodeId);
    return i < count_ ? &nodes_[i] : nullptr;
}

bool NodeTable::guarded(const NodeStatus& node) noexcept {
    return node.guardTime > Duration::zero() && node.lifeTimeFactor > 0;
}

void NodeTable::onFrame(const CanFrame& frame, TimePoint now) noexcept {
    if (!isUsable(frame) || frame.len != 1 || frame.id < 0x701 || frame.id > 0x77F) {
        return;
    }
    const std::uint8_t slot = slot_[frame.id - 0x700];
    if (slot == kNoSlot) {
        return;
    }
    NodeStatus& node = nodes_[slot];
    const std::uint8_t value = frame.data[0];

    if (value == 0) {  // boot-up
        node.state = NmtState::PreOperational;
        node.monitoring = false;
        node.heartbeatLost = false;
        node.toggleKnown = false;
        node.lastHeartbeat = now;
        emit(NodeEventKind::BootUp, node);
        return;
    }
    if (guarded(node)) {
        onGuardReply(node, value, now);
        return;
    }
    NmtState state;
    if (!knownState(value, state)) {
        return;  // malformed
    }

    node.lastHeartbeat = now;
    node.monitoring = node.heartbeatTimeout > Duration::zero();
    if (node.heartbeatLost) {
        node.heartbeatLost = false;
        emit(NodeEventKind::HeartbeatResumed, node);
    }
    if (node.state != state) {
        node.state = state;
        emit(NodeEventKind::StateChanged, node);
    }
}

void NodeTable::onGuardReply(NodeStatus& node, std::uint8_t value, TimePoint now) noexcept {
    NmtState state;
    const auto toggle = static_cast<std::uint8_t>(value >> 7);
    if (!knownState(static_cast<std::uint8_t>(value & 0x7F), state)) {
        return;  // malformed
    }
    if (node.toggleKnown && toggle == node.toggle) {
        return;  // toggle did not alternate: handled as not received
    }
    node.toggleKnown = true;
    node.toggle = toggle;
    node.lastHeartbeat = now;
    if (node.guardLost) {
        node.guardLost = false;
        emit(NodeEventKind::GuardResumed, node);
    }
    if (node.state != state) {
        node.state = state;
        emit(NodeEventKind::StateChanged, node);
    }
}

void NodeTable::onEmcy(const EmcyEvent& e) noexcept {
    const std::size_t i = indexOf(e.nodeId);
    if (i < count_) {
        nodes_[i].lastEmcyCode = e.errorCode;
        nodes_[i].lastErrorRegister = e.errorRegister;
        ++nodes_[i].emcyCount;
    }
}

void NodeTable::guard(NodeStatus& node, TimePoint now) noexcept {
    if (!node.guardStarted) {
        node.guardStarted = true;
        node.guardDue = now;
        node.lastHeartbeat = now;  // the life time runs from the first request
    }
    if (node.guardPending || now >= node.guardDue) {
        if (!node.guardPending) {
            node.guardDue += node.guardTime;
            if (node.guardDue <= now) {
                node.guardDue = now + node.guardTime;  // fell behind: no burst
            }
        }
        CanFrame f;
        f.id = 0x700u + node.nodeId;
        f.rtr = true;
        f.len = 1;
        node.guardPending = !bus_->send(f);
    }
    if (!node.guardLost && now - node.lastHeartbeat >= node.guardTime * node.lifeTimeFactor) {
        node.guardLost = true;
        node.state = NmtState::Unknown;
        emit(NodeEventKind::GuardLost, node);
    }
}

void NodeTable::tick(TimePoint now) noexcept {
    for (std::size_t i = 0; i < count_; ++i) {
        NodeStatus& node = nodes_[i];
        if (guarded(node) && bus_ != nullptr) {
            guard(node, now);
            continue;
        }
        if (node.monitoring && !node.heartbeatLost && now - node.lastHeartbeat >= node.heartbeatTimeout) {
            node.heartbeatLost = true;
            node.state = NmtState::Unknown;
            emit(NodeEventKind::HeartbeatLost, node);
        }
    }
}

TimePoint NodeTable::nextDeadline(TimePoint now) const noexcept {
    TimePoint d = TimePoint::max();
    const auto earlier = [&d](TimePoint t) { d = t < d ? t : d; };
    for (std::size_t i = 0; i < count_; ++i) {
        const NodeStatus& node = nodes_[i];
        if (guarded(node) && bus_ != nullptr) {
            if (!node.guardStarted || node.guardPending) {
                earlier(now);
                continue;
            }
            earlier(node.guardDue);
            if (!node.guardLost) {
                earlier(node.lastHeartbeat + node.guardTime * node.lifeTimeFactor);
            }
        } else if (node.monitoring && !node.heartbeatLost) {
            earlier(node.lastHeartbeat + node.heartbeatTimeout);
        }
    }
    return d;
}

void NodeTable::emit(NodeEventKind kind, const NodeStatus& node) noexcept {
    if (onEvent_ != nullptr) {
        NodeEvent e;
        e.kind = kind;
        e.nodeId = node.nodeId;
        e.state = node.state;
        onEvent_(context_, e);
    }
}

} // namespace canopen_conductor
