#pragma once

#include "canopen_conductor/can_bus.hpp"
#include "canopen_conductor/emcy.hpp"

#include <cstddef>
#include <cstdint>

namespace canopen_conductor {

// NMT state of a slave as reported by its heartbeat (CiA 301 figure 45).
enum class NmtState : std::uint8_t {
    Initialising = 0,  // a Device before its boot-up
    Stopped = 4,
    Operational = 5,
    PreOperational = 127,
    Unknown = 0xFF,  // no heartbeat yet, or heartbeat lost
};

struct NodeConfig {
    std::uint8_t nodeId = 0;      // 1..127
    Duration heartbeatTimeout{};  // consumer heartbeat time (1016h); 0 = do not monitor
    // COB-IDs for slaves that changed them; 0 = pre-defined connection set.
    std::uint32_t sdoRequestId = 0;   // client -> server, default 600h + node
    std::uint32_t sdoResponseId = 0;  // server -> client, default 580h + node
    std::uint32_t emcyId = 0;         // default 80h + node
    // Node guarding by the master; off when either is 0. Not together
    // with heartbeatTimeout.
    Duration guardTime{};
    std::uint8_t lifeTimeFactor = 0;
};

struct NodeStatus {
    std::uint8_t nodeId = 0;
    NmtState state = NmtState::Unknown;
    bool monitoring = false;      // heartbeat timeout check running (after first heartbeat)
    bool heartbeatLost = false;
    TimePoint lastHeartbeat{};    // last heartbeat, boot-up or accepted guarding reply
    Duration heartbeatTimeout{};

    // Node guarding.
    Duration guardTime{};
    std::uint8_t lifeTimeFactor = 0;
    bool guardLost = false;
    bool guardStarted = false;
    bool guardPending = false;    // request waiting for room in the send queue
    bool toggleKnown = false;     // a reply has been accepted since start / boot-up
    std::uint8_t toggle = 0;      // toggle bit of the last accepted reply
    TimePoint guardDue{};

    // Last emergency message.
    std::uint16_t lastEmcyCode = 0;
    std::uint8_t lastErrorRegister = 0;
    std::uint32_t emcyCount = 0;
};

enum class NodeEventKind {
    BootUp,            // boot-up frame; state is now PreOperational
    StateChanged,      // heartbeat / guarding reply with a new state
    HeartbeatLost,     // timeout; state is now Unknown
    HeartbeatResumed,  // first heartbeat after HeartbeatLost
    GuardLost,         // no guarding reply within the node life time; state Unknown
    GuardResumed,      // first accepted guarding reply after GuardLost
};

struct NodeEvent {
    NodeEventKind kind = NodeEventKind::StateChanged;
    std::uint8_t nodeId = 0;
    NmtState state = NmtState::Unknown;
};

// Called on the I/O thread. Must only signal.
using NodeEventFn = void (*)(void* context, const NodeEvent& event) noexcept;

// Slaves: heartbeat consumer, node guarding master and NMT state tracking.
class NodeTable {
public:
    // `storage` holds `capacity` entries and must outlive the table; the
    // first `count` are filled from `configs`. `bus` is needed only for
    // node guarding (may be null otherwise). Returns false for an invalid
    // config (see valid()) or count > capacity.
    bool init(NodeStatus* storage, std::size_t capacity, const NodeConfig* configs, std::size_t count,
              NodeEventFn onEvent, void* context, ICanBus* bus = nullptr) noexcept;
    bool init(NodeStatus* storage, const NodeConfig* configs, std::size_t count, NodeEventFn onEvent,
              void* context) noexcept {
        return init(storage, count, configs, count, onEvent, context);
    }

    // Node-ids 1..127 and unique, timeouts not negative, not heartbeat and
    // guarding on the same node. COB-IDs are checked by the Master.
    static bool valid(const NodeConfig* configs, std::size_t count) noexcept;
    static bool valid(const NodeConfig& config) noexcept;

    // Add a node at index size(). False if full, invalid or already there.
    bool add(const NodeConfig& config) noexcept;
    // Remove a node. The last node moves into its place. Returns the index
    // the node had, or size() (before the call) if it was not there.
    std::size_t remove(std::uint8_t nodeId) noexcept;

    void onFrame(const CanFrame& frame, TimePoint now) noexcept;
    void onEmcy(const EmcyEvent& event) noexcept;
    void tick(TimePoint now) noexcept;

    // Earliest timeout or guarding request; `now` if a request waits for
    // room in the send queue; TimePoint::max() if none.
    TimePoint nextDeadline(TimePoint now) const noexcept;
    TimePoint nextDeadline() const noexcept { return nextDeadline(TimePoint::min()); }

    // nullptr if the node is not in the table.
    const NodeStatus* find(std::uint8_t nodeId) const noexcept;
    // Index of the node in storage, or size() if not there.
    std::size_t indexOf(std::uint8_t nodeId) const noexcept;
    std::size_t size() const noexcept { return count_; }
    std::size_t capacity() const noexcept { return capacity_; }
    const NodeStatus& at(std::size_t i) const noexcept { return nodes_[i]; }

private:
    void emit(NodeEventKind kind, const NodeStatus& node) noexcept;
    void onGuardReply(NodeStatus& node, std::uint8_t value, TimePoint now) noexcept;
    void guard(NodeStatus& node, TimePoint now) noexcept;
    static bool guarded(const NodeStatus& node) noexcept;

    static constexpr std::uint8_t kNoSlot = 0xFF;

    NodeStatus* nodes_ = nullptr;
    std::size_t count_ = 0;
    std::size_t capacity_ = 0;
    std::uint8_t slot_[128] = {};  // node-id -> index into nodes_, or kNoSlot
    NodeEventFn onEvent_ = nullptr;
    void* context_ = nullptr;
    ICanBus* bus_ = nullptr;
};

} // namespace canopen_conductor
