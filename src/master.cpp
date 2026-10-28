#include "canopen_conductor/master.hpp"

#include <cstddef>
#include <cstdint>
#include <new>
#include <type_traits>

namespace canopen_conductor {

// Nothing in the block needs a destructor.
static_assert(std::is_trivially_destructible_v<Master>);
static_assert(std::is_trivially_destructible_v<NodeStatus>);
static_assert(std::is_trivially_destructible_v<SdoClient>);
static_assert(std::is_trivially_destructible_v<LssMaster>);

namespace {

constexpr std::size_t kCobIds = 2048;

LssConfig lssConfig(const MasterConfig& c) noexcept {
    LssConfig l;
    l.timeout = c.lssTimeout;
    l.fastscanWindow = c.lssFastscanWindow;
    l.onDone = c.onLss;
    l.context = c.context;
    return l;
}

std::size_t alignUp(std::size_t n, std::size_t a) noexcept {
    return (n + a - 1) / a * a;
}

std::size_t capacityOf(std::size_t count, std::size_t max) noexcept {
    return max == 0 ? count : max;
}

// CAN-IDs this node's frames come in on (routed), and its SDO request ID.
struct NodeIds {
    std::uint32_t request;
    std::uint32_t response;
    std::uint32_t emcy;
};

NodeIds idsOf(const NodeConfig& c) noexcept {
    return {c.sdoRequestId != 0 ? c.sdoRequestId : 0x600u + c.nodeId,
            c.sdoResponseId != 0 ? c.sdoResponseId : 0x580u + c.nodeId,
            c.emcyId != 0 ? c.emcyId : 0x080u + c.nodeId};
}

// CAN-IDs that can never be routed to a node or PDO.
bool reserved(std::uint32_t id, std::uint32_t syncId, std::uint32_t timeId) noexcept {
    return id == 0 || id > 0x7FF || (id >= 0x701 && id <= 0x77F) || id == 0x7E4 || id == 0x7E5 || id == syncId ||
           id == timeId;
}

std::size_t mappedBytes(const PdoEntry* entries, std::size_t count) noexcept {
    unsigned bits = 0;
    for (std::size_t k = 0; k < count; ++k) {
        bits += pdoBits(entries[k].type);
    }
    return (bits + 7) / 8;
}

bool validPdoShape(const PdoConfig& p) noexcept {
    if (p.timeout < Duration::zero() || !validPdoCobId(p.cobId)) {
        return false;
    }
    // MPDOs carry a different object each time: no deadline monitoring.
    return p.mpdo ? p.entryCount == 0 && p.timeout == Duration::zero() : validPdoEntries(p.entries, p.entryCount);
}

// Dry run of create(): every node and PDO valid, every routed CAN-ID unique.
bool validConfig(const MasterConfig& c) noexcept {
    const bool sdoOk = c.sdo.blockSize >= 1 && c.sdo.blockSize <= 127 && c.sdo.maxBurst >= 1 &&
                       c.sdo.maxBurst <= 127;
    const bool nmtOk = c.nmt.heartbeatNodeId <= 127 && c.nmt.heartbeatPeriod >= Duration::zero();
    const bool lssOk = c.lssTimeout > Duration::zero() && c.lssFastscanWindow > Duration::zero();
    const std::size_t maxNodes = capacityOf(c.nodeCount, c.maxNodes);
    const std::size_t maxPdos = capacityOf(c.pdoCount, c.maxPdos);
    if (!sdoOk || !nmtOk || !lssOk || !SyncProducer::valid(c.sync) || c.timeCobId > 0x7FF ||
        !NodeTable::valid(c.nodes, c.nodeCount) || maxNodes > 127 || c.nodeCount > maxNodes ||
        maxPdos > kCobIds || c.pdoCount > maxPdos || (c.pdoCount > 0 && c.pdos == nullptr)) {
        return false;
    }
    bool used[kCobIds] = {};
    const auto claim = [&](std::uint32_t id) {
        if (reserved(id, c.sync.cobId, c.timeCobId) || used[id]) {
            return false;
        }
        used[id] = true;
        return true;
    };
    for (std::size_t i = 0; i < c.nodeCount; ++i) {
        const NodeIds ids = idsOf(c.nodes[i]);
        if (ids.request > 0x7FF || !claim(ids.response) || !claim(ids.emcy)) {
            return false;
        }
    }
    for (std::size_t i = 0; i < c.pdoCount; ++i) {
        const PdoConfig& p = c.pdos[i];
        bool nodeKnown = false;
        for (std::size_t k = 0; k < c.nodeCount; ++k) {
            nodeKnown = nodeKnown || c.nodes[k].nodeId == p.nodeId;
        }
        if (!nodeKnown || !validPdoShape(p) || !claim(p.cobId)) {
            return false;
        }
    }
    return true;
}

} // namespace

// Where each part sits inside the host's block.
struct Master::Layout {
    std::size_t statuses;
    std::size_t clients;
    std::size_t ids;
    std::size_t slots;
    std::size_t route;
    std::size_t total;
};

Master::Layout Master::layoutFor(const MasterConfig& c) noexcept {
    const std::size_t maxNodes = capacityOf(c.nodeCount, c.maxNodes);
    const std::size_t maxPdos = capacityOf(c.pdoCount, c.maxPdos);
    Layout l{};
    l.statuses = alignUp(sizeof(Master), alignof(NodeStatus));
    l.clients = alignUp(l.statuses + maxNodes * sizeof(NodeStatus), alignof(SdoClient));
    l.ids = alignUp(l.clients + maxNodes * sizeof(SdoClient), alignof(std::uint32_t));
    l.slots = alignUp(l.ids + maxNodes * 3 * sizeof(std::uint32_t), alignof(PdoSlot));
    l.route = alignUp(l.slots + maxPdos * sizeof(PdoSlot), alignof(std::uint16_t));
    l.total = l.route + kCobIds * sizeof(std::uint16_t);
    return l;
}

std::size_t Master::requiredBytes(const MasterConfig& config) noexcept {
    return validConfig(config) ? layoutFor(config).total : 0;
}

Master* Master::create(void* memory, std::size_t size, const MasterConfig& config, ICanBus& bus) noexcept {
    const std::size_t need = requiredBytes(config);
    if (need == 0 || memory == nullptr || size < need ||
        reinterpret_cast<std::uintptr_t>(memory) % alignof(std::max_align_t) != 0) {
        return nullptr;
    }
    const Layout l = layoutFor(config);
    auto* base = static_cast<unsigned char*>(memory);
    const std::size_t maxNodes = capacityOf(config.nodeCount, config.maxNodes);

    Master* m = new (base) Master(bus, config);
    auto* statuses = reinterpret_cast<NodeStatus*>(base + l.statuses);
    for (std::size_t i = 0; i < maxNodes; ++i) {
        new (statuses + i) NodeStatus{};
    }
    m->nodes_.init(statuses, maxNodes, nullptr, 0, config.onNodeEvent, config.context, &bus);
    m->sdo_ = reinterpret_cast<SdoClient*>(base + l.clients);
    m->sdoIds_ = reinterpret_cast<std::uint32_t*>(base + l.ids);
    m->pdos_ = reinterpret_cast<PdoSlot*>(base + l.slots);
    m->maxPdos_ = capacityOf(config.pdoCount, config.maxPdos);
    m->route_ = reinterpret_cast<std::uint16_t*>(base + l.route);
    for (std::size_t i = 0; i < kCobIds; ++i) {
        m->route_[i] = RouteNone;
    }
    for (std::size_t i = 0; i < config.nodeCount; ++i) {
        m->addNode(config.nodes[i]);
    }
    for (std::size_t i = 0; i < config.pdoCount; ++i) {
        m->addPdo(config.pdos[i]);
    }
    return m;
}

Master::Master(ICanBus& bus, const MasterConfig& config) noexcept
    : bus_(&bus), nmt_(bus, config.nmt), sync_(bus, config.sync), lss_(bus, lssConfig(config)),
      sdoConfig_(config.sdo), onSdoDone_(config.onSdoDone), onEmcy_(config.onEmcy), onPdo_(config.onPdo),
      onMpdo_(config.onMpdo), context_(config.context), syncCobId_(config.sync.cobId),
      timeCobId_(config.timeCobId) {}

// ---------------------------------------------------------------------------
// Nodes and PDOs

SdoClientConfig Master::clientConfig(const NodeConfig& node) const noexcept {
    const NodeIds ids = idsOf(node);
    SdoClientConfig c;
    c.nodeId = node.nodeId;
    c.requestId = ids.request;
    c.responseId = ids.response;
    c.blockSize = sdoConfig_.blockSize;
    c.switchThreshold = sdoConfig_.switchThreshold;
    c.maxBurst = sdoConfig_.maxBurst;
    c.onDone = onSdoDone_;
    c.context = context_;
    return c;
}

void Master::setRoute(std::uint32_t id, Route kind, std::size_t slot) noexcept {
    route_[id] = kind == RouteNone ? std::uint16_t{0} : static_cast<std::uint16_t>((static_cast<std::size_t>(kind) << 13) | slot);
}

// Point (or clear) the routes of the node in slot `index`.
void Master::routeNode(std::size_t index, bool on) noexcept {
    const std::uint32_t* ids = &sdoIds_[index * 3];
    setRoute(ids[1], on ? RouteSdo : RouteNone, index);
    setRoute(ids[2], on ? RouteEmcy : RouteNone, index);
}

bool Master::addNode(const NodeConfig& config) noexcept {
    const NodeIds ids = idsOf(config);
    const auto freeId = [this](std::uint32_t id) {
        return !reserved(id, syncCobId_, timeCobId_) && route_[id] == RouteNone;
    };
    if (!NodeTable::valid(config) || nodes_.find(config.nodeId) != nullptr || nodes_.size() >= nodes_.capacity() ||
        ids.request > 0x7FF || ids.response == ids.emcy || !freeId(ids.response) || !freeId(ids.emcy)) {
        return false;
    }
    const std::size_t index = nodes_.size();
    nodes_.add(config);
    new (&sdo_[index]) SdoClient(*bus_, clientConfig(config));
    sdoIds_[index * 3] = ids.request;
    sdoIds_[index * 3 + 1] = ids.response;
    sdoIds_[index * 3 + 2] = ids.emcy;
    routeNode(index, true);
    return true;
}

bool Master::removeNode(std::uint8_t nodeId) noexcept {
    const std::size_t index = nodes_.indexOf(nodeId);
    if (index >= nodes_.size()) {
        return false;
    }
    sdo_[index].cancel(sdo_abort::General);
    for (std::size_t i = pdoCount_; i > 0; --i) {
        if (pdos_[i - 1].nodeId == nodeId) {
            removePdoAt(i - 1);
        }
    }
    routeNode(index, false);
    const std::size_t last = nodes_.size() - 1;
    nodes_.remove(nodeId);
    if (index != last) {
        // The last node moved into this slot: move its client and routes too.
        routeNode(last, false);
        sdo_[index] = sdo_[last];
        for (std::size_t k = 0; k < 3; ++k) {
            sdoIds_[index * 3 + k] = sdoIds_[last * 3 + k];
        }
        routeNode(index, true);
    }
    return true;
}

bool Master::addPdo(const PdoConfig& p) noexcept {
    if (pdoCount_ >= maxPdos_ || nodes_.find(p.nodeId) == nullptr || !validPdoShape(p) ||
        reserved(p.cobId, syncCobId_, timeCobId_) || route_[p.cobId] != RouteNone) {
        return false;
    }
    PdoSlot* slot = new (&pdos_[pdoCount_]) PdoSlot{};
    for (std::size_t k = 0; k < p.entryCount; ++k) {
        slot->entries[k] = p.entries[k];
    }
    slot->entryCount = p.entryCount;
    slot->nodeId = p.nodeId;
    slot->cobId = p.cobId;
    slot->mpdo = p.mpdo;
    slot->timeout = p.timeout;
    setRoute(p.cobId, RoutePdo, pdoCount_);
    ++pdoCount_;
    return true;
}

void Master::removePdoAt(std::size_t i) noexcept {
    setRoute(pdos_[i].cobId, RouteNone, 0);
    const std::size_t last = pdoCount_ - 1;
    if (i != last) {
        pdos_[i] = pdos_[last];
        setRoute(pdos_[i].cobId, RoutePdo, i);
    }
    --pdoCount_;
}

bool Master::removePdo(std::uint16_t cobId) noexcept {
    if (cobId >= kCobIds || (route_[cobId] >> 13) != RoutePdo) {
        return false;
    }
    removePdoAt(route_[cobId] & 0x1FFF);
    return true;
}

SdoClient* Master::client(std::uint8_t nodeId) const noexcept {
    const std::size_t i = nodes_.indexOf(nodeId);
    return i < nodes_.size() ? &sdo_[i] : nullptr;
}

// ---------------------------------------------------------------------------
// Events

void Master::onFrame(const CanFrame& frame, TimePoint now) noexcept {
    if (!isUsable(frame)) {
        return;
    }
    if (frame.id == 0x7E4) {
        lss_.onFrame(frame, now);
        return;
    }
    const std::uint16_t r = route_[frame.id];
    const std::size_t slot = r & 0x1FFF;
    switch (r >> 13) {
    case RoutePdo:
        onPdoFrame(pdos_[slot], frame, now);
        return;
    case RouteSdo:
        sdo_[slot].onFrame(frame, now);
        return;
    case RouteEmcy: {
        EmcyEvent e;
        if (decodeEmcy(frame, e)) {
            e.nodeId = nodes_.at(slot).nodeId;  // the CAN-ID may be a remapped one
            nodes_.onEmcy(e);
            if (onEmcy_ != nullptr) {
                onEmcy_(context_, e);
            }
        }
        return;
    }
    default:
        break;
    }
    if (frame.id >= 0x701 && frame.id <= 0x77F) {
        nodes_.onFrame(frame, now);
    } else if (frame.id == syncCobId_ && !sync_.enabled()) {
        ++syncCount_;
    }
}

void Master::onPdoFrame(PdoSlot& slot, const CanFrame& frame, TimePoint now) noexcept {
    slot.lastRx = now;
    slot.monitoring = slot.timeout > Duration::zero();
    slot.timedOut = false;
    if (slot.mpdo) {
        if (onMpdo_ == nullptr || frame.len != 8) {
            return;
        }
        MpdoEvent e;
        e.nodeId = slot.nodeId;
        e.cobId = slot.cobId;
        e.destination = (frame.data[0] & 0x80) != 0;
        e.address = static_cast<std::uint8_t>(frame.data[0] & 0x7F);
        e.index = static_cast<std::uint16_t>(frame.data[1] | (frame.data[2] << 8));
        e.subIndex = frame.data[3];
        for (int i = 0; i < 4; ++i) {
            e.data[i] = frame.data[4 + i];
        }
        e.syncCount = syncCount_;
        onMpdo_(context_, e);
        return;
    }
    if (onPdo_ != nullptr) {
        reportPdo(slot, decodePdo(slot.entries, slot.entryCount, frame, pdoValues_), false);
    }
}

void Master::reportPdo(const PdoSlot& slot, int count, bool timeout) noexcept {
    PdoEvent e;
    e.nodeId = slot.nodeId;
    e.cobId = slot.cobId;
    e.lengthError = !timeout && count < 0;
    e.timeout = timeout;
    e.values = pdoValues_;
    e.count = count < 0 ? 0 : static_cast<std::uint8_t>(count);
    e.syncCount = syncCount_;
    onPdo_(context_, e);
}

void Master::tick(TimePoint now) noexcept {
    if (sync_.tick(now)) {
        ++syncCount_;
    }
    nmt_.tick(now);
    lss_.tick(now);
    for (std::size_t i = 0; i < nodes_.size(); ++i) {
        sdo_[i].tick(now);
    }
    nodes_.tick(now);
    for (std::size_t i = 0; i < pdoCount_; ++i) {
        PdoSlot& s = pdos_[i];
        if (s.monitoring && !s.timedOut && now - s.lastRx >= s.timeout) {
            s.timedOut = true;
            if (onPdo_ != nullptr) {
                reportPdo(s, 0, true);
            }
        }
    }
}

TimePoint Master::nextDeadline(TimePoint now) const noexcept {
    TimePoint d = sync_.nextDeadline(now);
    const auto earlier = [&d](TimePoint t) { d = t < d ? t : d; };
    earlier(nmt_.nextDeadline(now));
    earlier(lss_.nextDeadline(now));
    earlier(nodes_.nextDeadline(now));
    for (std::size_t i = 0; i < nodes_.size(); ++i) {
        earlier(sdo_[i].nextDeadline(now));
    }
    for (std::size_t i = 0; i < pdoCount_; ++i) {
        if (pdos_[i].monitoring && !pdos_[i].timedOut) {
            earlier(pdos_[i].lastRx + pdos_[i].timeout);
        }
    }
    return d;
}

// ---------------------------------------------------------------------------
// Requests

SdoStart Master::sdoUpload(std::uint8_t nodeId, const SdoUpload& request, TimePoint now) noexcept {
    SdoClient* c = client(nodeId);
    return c != nullptr ? c->upload(request, now) : SdoStart::InvalidArgument;
}

SdoStart Master::sdoDownload(std::uint8_t nodeId, const SdoDownload& request, TimePoint now) noexcept {
    SdoClient* c = client(nodeId);
    return c != nullptr ? c->download(request, now) : SdoStart::InvalidArgument;
}

void Master::sdoCancel(std::uint8_t nodeId, std::uint32_t abortCode) noexcept {
    if (SdoClient* c = client(nodeId)) {
        c->cancel(abortCode);
    }
}

bool Master::sdoBusy(std::uint8_t nodeId) const noexcept {
    const SdoClient* c = client(nodeId);
    return c != nullptr && c->busy();
}

bool Master::nmtCommand(NmtCommand command, std::uint8_t nodeId) noexcept {
    return nmt_.command(command, nodeId);
}

void Master::setHeartbeatState(std::uint8_t state) noexcept {
    nmt_.setHeartbeatState(state);
}

bool Master::requestPdo(std::uint16_t cobId) noexcept {
    if (cobId >= kCobIds || (route_[cobId] >> 13) != RoutePdo) {
        return false;
    }
    const PdoSlot& s = pdos_[route_[cobId] & 0x1FFF];
    CanFrame f;
    f.id = cobId;
    f.rtr = true;
    f.len = static_cast<std::uint8_t>(s.mpdo ? 8 : mappedBytes(s.entries, s.entryCount));
    return bus_->send(f);
}

bool Master::sendTime(std::uint32_t ms, std::uint16_t days) noexcept {
    CanFrame f;
    f.id = timeCobId_;
    f.len = 6;
    ms &= 0x0FFFFFFFu;
    for (int i = 0; i < 4; ++i) {
        f.data[i] = static_cast<std::uint8_t>(ms >> (8 * i));
    }
    f.data[4] = static_cast<std::uint8_t>(days);
    f.data[5] = static_cast<std::uint8_t>(days >> 8);
    return bus_->send(f);
}

const NodeStatus* Master::node(std::uint8_t nodeId) const noexcept {
    return nodes_.find(nodeId);
}

} // namespace canopen_conductor
