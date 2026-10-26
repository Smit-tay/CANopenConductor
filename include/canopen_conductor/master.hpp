#pragma once

#include "canopen_conductor/can_bus.hpp"
#include "canopen_conductor/emcy.hpp"
#include "canopen_conductor/lss_master.hpp"
#include "canopen_conductor/nmt_master.hpp"
#include "canopen_conductor/node_table.hpp"
#include "canopen_conductor/pdo.hpp"
#include "canopen_conductor/sdo_client.hpp"
#include "canopen_conductor/sync_producer.hpp"

#include <cstddef>
#include <cstdint>

namespace canopen_conductor {

// SDO settings shared by every node's client.
struct MasterSdoConfig {
    std::uint8_t blockSize = 127;      // block upload segments per block, 1..127
    std::uint8_t switchThreshold = 21; // block upload pst
    std::uint8_t maxBurst = 8;         // block download frames per call, 1..127
};

struct MasterConfig {
    const NodeConfig* nodes = nullptr;  // copied by create()
    std::size_t nodeCount = 0;
    std::size_t maxNodes = 0;           // capacity for addNode; 0 = nodeCount
    NmtMasterConfig nmt{};
    SyncConfig sync{};                  // own SYNC producer; its cobId is also the SYNC consumed when off
    MasterSdoConfig sdo{};
    const PdoConfig* pdos = nullptr;    // slave TPDOs to decode; copied by create()
    std::size_t pdoCount = 0;
    std::size_t maxPdos = 0;            // capacity for addPdo; 0 = pdoCount
    std::uint32_t timeCobId = 0x100;    // TIME producer (sendTime)
    Duration lssTimeout = Duration{1'000'000};        // LSS confirmed services
    Duration lssFastscanWindow = Duration{100'000};   // LSS fastscan step

    // Callbacks run on the I/O thread and must only signal. Any may be null.
    SdoDoneFn onSdoDone = nullptr;
    NodeEventFn onNodeEvent = nullptr;
    EmcyFn onEmcy = nullptr;
    PdoFn onPdo = nullptr;
    MpdoFn onMpdo = nullptr;
    LssDoneFn onLss = nullptr;
    void* context = nullptr;
};

// The CANopen master. All calls from the host's I/O thread.
class Master {
public:
    // Bytes needed for `config`, or 0 if the config is invalid.
    static std::size_t requiredBytes(const MasterConfig& config) noexcept;

    // Build the master inside `memory` (aligned to alignof(std::max_align_t),
    // at least requiredBytes(config) long). nullptr on any error. The block
    // must stay valid while the master is used; nothing needs destroying.
    static Master* create(void* memory, std::size_t size, const MasterConfig& config, ICanBus& bus) noexcept;

    void onFrame(const CanFrame& frame, TimePoint now) noexcept;
    void tick(TimePoint now) noexcept;

    // Earliest time tick() has work to do if no frame arrives first. The
    // host can wait for a frame until then instead of polling. Returns
    // `now` while a frame waits for room in the send queue (retry pace is
    // the host's choice), a past time if something is already due, and
    // TimePoint::max() if nothing is timed.
    TimePoint nextDeadline(TimePoint now) const noexcept;

    // Nodes and PDOs at run time, within maxNodes / maxPdos. False if full,
    // invalid, or a COB-ID is already in use. Removing a node aborts its
    // SDO transfer (abort 08000000h sent, ClientAbort reported) and removes
    // its PDOs.
    bool addNode(const NodeConfig& config) noexcept;
    bool removeNode(std::uint8_t nodeId) noexcept;
    bool addPdo(const PdoConfig& config) noexcept;
    bool removePdo(std::uint16_t cobId) noexcept;

    // SDO to a configured node. InvalidArgument for an unknown node.
    SdoStart sdoUpload(std::uint8_t nodeId, const SdoUpload& request, TimePoint now) noexcept;
    SdoStart sdoDownload(std::uint8_t nodeId, const SdoDownload& request, TimePoint now) noexcept;
    void sdoCancel(std::uint8_t nodeId, std::uint32_t abortCode) noexcept;
    bool sdoBusy(std::uint8_t nodeId) const noexcept;

    // NMT command; node 0 = all. See NmtMaster::command.
    bool nmtCommand(NmtCommand command, std::uint8_t nodeId) noexcept;
    // State byte of the master's own heartbeat (4, 5 or 127).
    void setHeartbeatState(std::uint8_t state) noexcept;

    // Remote request for a configured PDO (slave transmission type
    // 252/253). False if the PDO is unknown or the send queue is full.
    bool requestPdo(std::uint16_t cobId) noexcept;

    // TIME: milliseconds after midnight and days since 1984-01-01. False if
    // the send queue is full.
    bool sendTime(std::uint32_t msAfterMidnight, std::uint16_t days) noexcept;

    // SYNCs sent by the own producer, or seen on the bus while it is off.
    std::uint32_t syncCount() const noexcept { return syncCount_; }

    // Tracked state of a configured node, or nullptr.
    const NodeStatus* node(std::uint8_t nodeId) const noexcept;
    std::size_t nodeCount() const noexcept { return nodes_.size(); }
    const NodeStatus& nodeAt(std::size_t i) const noexcept { return nodes_.at(i); }
    std::size_t pdoCount() const noexcept { return pdoCount_; }

    // LSS master for commissioning (frames on 7E4h are routed to it).
    LssMaster& lss() noexcept { return lss_; }

    Master(const Master&) = delete;
    Master& operator=(const Master&) = delete;

private:
    // One configured PDO.
    struct PdoSlot {
        PdoEntry entries[64];
        std::uint8_t entryCount;
        std::uint8_t nodeId;
        std::uint16_t cobId;
        bool mpdo;
        bool monitoring;  // deadline check running (after the first frame)
        bool timedOut;
        Duration timeout;
        TimePoint lastRx;
    };

    // What a CAN-ID is routed to: kind in bits 15..13, slot in bits 12..0.
    enum Route : std::uint16_t { RouteNone = 0, RoutePdo = 1, RouteSdo = 2, RouteEmcy = 3 };

    struct Layout;
    static Layout layoutFor(const MasterConfig& config) noexcept;

    Master(ICanBus& bus, const MasterConfig& config) noexcept;
    SdoClient* client(std::uint8_t nodeId) const noexcept;
    SdoClientConfig clientConfig(const NodeConfig& node) const noexcept;
    void setRoute(std::uint32_t id, Route kind, std::size_t slot) noexcept;
    void routeNode(std::size_t index, bool on) noexcept;
    void removePdoAt(std::size_t slot) noexcept;
    void onPdoFrame(PdoSlot& slot, const CanFrame& frame, TimePoint now) noexcept;
    void reportPdo(const PdoSlot& slot, int count, bool timeout) noexcept;

    ICanBus* bus_;
    NodeTable nodes_;
    NmtMaster nmt_;
    SyncProducer sync_;
    LssMaster lss_;
    MasterSdoConfig sdoConfig_;
    SdoDoneFn onSdoDone_;
    EmcyFn onEmcy_;
    PdoFn onPdo_;
    MpdoFn onMpdo_;
    void* context_;
    std::uint32_t syncCobId_;
    std::uint32_t timeCobId_;
    std::uint32_t syncCount_ = 0;
    SdoClient* sdo_ = nullptr;      // one per node slot, same order as nodes_
    std::uint32_t* sdoIds_ = nullptr;   // per node slot: request, response, emcy CAN-IDs (3 each)
    PdoSlot* pdos_ = nullptr;
    std::size_t pdoCount_ = 0;
    std::size_t maxPdos_ = 0;
    std::uint16_t* route_ = nullptr;  // 2048 entries
    PdoValue pdoValues_[64] = {};
};

} // namespace canopen_conductor
