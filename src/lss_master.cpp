#include "canopen_conductor/lss_master.hpp"

namespace canopen_conductor {

namespace {

constexpr std::uint32_t kRequestId = 0x7E5;
constexpr std::uint32_t kResponseId = 0x7E4;

constexpr std::uint8_t kSwitchGlobal = 0x04;
constexpr std::uint8_t kConfigureNodeId = 0x11;
constexpr std::uint8_t kConfigureBitTiming = 0x13;
constexpr std::uint8_t kActivateBitTiming = 0x15;
constexpr std::uint8_t kStore = 0x17;
constexpr std::uint8_t kSelectVendor = 0x40;  // 40h..43h: vendor, product, revision, serial
constexpr std::uint8_t kSelectConfirm = 0x44;
constexpr std::uint8_t kIdentifyConfirm = 0x4F;
constexpr std::uint8_t kFastscan = 0x51;
constexpr std::uint8_t kFastscanReset = 0x80;  // bit-checked value: reset and confirm

std::uint32_t getU32(const std::uint8_t* p) noexcept {
    return static_cast<std::uint32_t>(p[0]) | (static_cast<std::uint32_t>(p[1]) << 8) |
           (static_cast<std::uint32_t>(p[2]) << 16) | (static_cast<std::uint32_t>(p[3]) << 24);
}

} // namespace

LssMaster::LssMaster(ICanBus& bus, const LssConfig& config) noexcept : bus_(&bus), config_(config) {}

// ---------------------------------------------------------------------------
// Sending

void LssMaster::queue(std::uint8_t command, std::uint32_t value, std::uint8_t b5, std::uint8_t b6,
                      std::uint8_t b7) noexcept {
    CanFrame& f = queue_[(queueHead_ + queueCount_) % 4];
    f = CanFrame{};
    f.id = kRequestId;
    f.len = 8;
    f.data[0] = command;
    f.data[1] = static_cast<std::uint8_t>(value);
    f.data[2] = static_cast<std::uint8_t>(value >> 8);
    f.data[3] = static_cast<std::uint8_t>(value >> 16);
    f.data[4] = static_cast<std::uint8_t>(value >> 24);
    f.data[5] = b5;
    f.data[6] = b6;
    f.data[7] = b7;
    ++queueCount_;
    timerRunning_ = false;
}

void LssMaster::pump(TimePoint now) noexcept {
    while (queueCount_ > 0) {
        if (!bus_->send(queue_[queueHead_])) {
            return;
        }
        queueHead_ = static_cast<std::uint8_t>((queueHead_ + 1) % 4);
        --queueCount_;
    }
    if (phase_ != Phase::Idle && !timerRunning_) {
        timerRunning_ = true;  // the step's last frame just went out
        timerStart_ = now;
    }
}

LssStart LssMaster::canStart(Selection required1, Selection required2, unsigned frames) const noexcept {
    if (busy() || queueCount_ + frames > 4) {
        return LssStart::Busy;
    }
    if (selection_ != required1 && selection_ != required2) {
        return LssStart::WrongState;
    }
    return LssStart::Started;
}

void LssMaster::startConfirmed(LssOp op, std::uint8_t expect, TimePoint now) noexcept {
    result_ = LssResult{};
    result_.op = op;
    expect_ = expect;
    phase_ = Phase::Response;
    pump(now);
}

void LssMaster::finish(LssStatus status) noexcept {
    result_.status = status;
    phase_ = Phase::Idle;
    timerRunning_ = false;
    if (config_.onDone != nullptr) {
        config_.onDone(config_.context, result_);
    }
}

// ---------------------------------------------------------------------------
// Requests

LssStart LssMaster::switchGlobal(bool configuration, TimePoint now) noexcept {
    if (busy() || queueCount_ >= 4) {
        return LssStart::Busy;
    }
    queue(kSwitchGlobal, configuration ? 1u : 0u);
    selection_ = configuration ? Selection::Global : Selection::None;
    pump(now);
    return LssStart::Started;
}

LssStart LssMaster::activateBitTiming(std::uint16_t delayMs, TimePoint now) noexcept {
    const LssStart s = canStart(Selection::Global, Selection::Global, 1);
    if (s == LssStart::Started) {
        queue(kActivateBitTiming, delayMs);
        pump(now);
    }
    return s;
}

LssStart LssMaster::switchSelective(const LssAddress& a, TimePoint now) noexcept {
    const LssStart s = canStart(Selection::None, Selection::None, 4);
    if (s != LssStart::Started) {
        return s;
    }
    queue(kSelectVendor, a.vendorId);
    queue(kSelectVendor + 1, a.productCode);
    queue(kSelectVendor + 2, a.revision);
    queue(kSelectVendor + 3, a.serial);
    startConfirmed(LssOp::SwitchSelective, kSelectConfirm, now);
    return s;
}

LssStart LssMaster::configureNodeId(std::uint8_t nodeId, TimePoint now) noexcept {
    if ((nodeId < 1 || nodeId > 127) && nodeId != 0xFF) {
        return LssStart::InvalidArgument;
    }
    // Global mode only to unconfigure every slave at once (ref).
    const LssStart s = nodeId == 0xFF ? canStart(Selection::Selective, Selection::Global, 1)
                                      : canStart(Selection::Selective, Selection::Selective, 1);
    if (s == LssStart::Started) {
        queue(kConfigureNodeId, nodeId);
        startConfirmed(LssOp::ConfigureNodeId, kConfigureNodeId, now);
    }
    return s;
}

LssStart LssMaster::configureBitTiming(std::uint8_t tableIndex, TimePoint now) noexcept {
    if (tableIndex > 9 || tableIndex == 5) {
        return LssStart::InvalidArgument;
    }
    const LssStart s = canStart(Selection::Selective, Selection::Selective, 1);
    if (s == LssStart::Started) {
        queue(kConfigureBitTiming, static_cast<std::uint32_t>(tableIndex) << 8);  // byte 1: table 0, byte 2: index
        startConfirmed(LssOp::ConfigureBitTiming, kConfigureBitTiming, now);
    }
    return s;
}

LssStart LssMaster::store(TimePoint now) noexcept {
    const LssStart s = canStart(Selection::Selective, Selection::Selective, 1);
    if (s == LssStart::Started) {
        queue(kStore, 0);
        startConfirmed(LssOp::Store, kStore, now);
    }
    return s;
}

LssStart LssMaster::inquire(LssInquire what, TimePoint now) noexcept {
    const auto cs = static_cast<std::uint8_t>(what);
    if (cs < 0x5A || cs > 0x5E) {
        return LssStart::InvalidArgument;
    }
    const LssStart s = canStart(Selection::Selective, Selection::Global, 1);
    if (s == LssStart::Started) {
        queue(cs, 0);
        startConfirmed(LssOp::Inquire, cs, now);
    }
    return s;
}

LssStart LssMaster::fastscan(const LssFastscan& request, TimePoint now) noexcept {
    int skipped = 0;
    for (LssScan m : {request.product, request.revision, request.serial}) {
        skipped += m == LssScan::Skip ? 1 : 0;
    }
    if (request.vendor == LssScan::Skip || skipped > 2) {
        return LssStart::InvalidArgument;
    }
    const LssStart s = canStart(Selection::None, Selection::None, 1);
    if (s != LssStart::Started) {
        return s;
    }
    result_ = LssResult{};
    result_.op = LssOp::Fastscan;
    fs_ = request;
    phase_ = Phase::FastscanCheck;
    sendFastscan(0, kFastscanReset, 0, 0, now);
    return s;
}

// ---------------------------------------------------------------------------
// Events

void LssMaster::onFrame(const CanFrame& frame, TimePoint now) noexcept {
    (void)now;
    if (frame.id != kResponseId || !isUsable(frame) || frame.len != 8 || !busy()) {
        return;
    }
    const std::uint8_t* d = frame.data;
    if (phase_ != Phase::Response) {
        // Counted when the window ends; only once this step's request is on the bus.
        if (d[0] == kIdentifyConfirm && timerRunning_) {
            acked_ = true;
        }
        return;
    }
    if (d[0] != expect_) {
        return;  // not our answer; keep waiting
    }
    switch (result_.op) {
    case LssOp::SwitchSelective:
        selection_ = Selection::Selective;
        finish(LssStatus::Ok);
        break;
    case LssOp::ConfigureNodeId:
    case LssOp::ConfigureBitTiming:
    case LssOp::Store:
        result_.errorCode = d[1];
        result_.manufacturerError = d[1] == 0xFF ? d[2] : 0;
        finish(d[1] == 0 ? LssStatus::Ok : LssStatus::Rejected);
        break;
    case LssOp::Inquire:
        result_.value = expect_ == static_cast<std::uint8_t>(LssInquire::NodeId) ? d[1] : getU32(&d[1]);
        finish(LssStatus::Ok);
        break;
    case LssOp::Fastscan:
        break;
    }
}

void LssMaster::tick(TimePoint now) noexcept {
    pump(now);
    if (!busy() || !timerRunning_) {
        return;
    }
    const Duration elapsed = now - timerStart_;
    if (phase_ == Phase::Response) {
        if (elapsed >= config_.timeout) {
            if (result_.op == LssOp::SwitchSelective) {
                selection_ = Selection::None;
            }
            finish(LssStatus::Timeout);
        }
    } else if (elapsed >= config_.fastscanWindow) {
        fastscanStepDone(now);
    }
}

TimePoint LssMaster::nextDeadline(TimePoint now) const noexcept {
    if (queueCount_ > 0) {
        return now;
    }
    if (!busy() || !timerRunning_) {
        return TimePoint::max();
    }
    return timerStart_ + (phase_ == Phase::Response ? config_.timeout : config_.fastscanWindow);
}

// ---------------------------------------------------------------------------
// Fastscan

LssScan LssMaster::mode(std::uint8_t part) const noexcept {
    switch (part) {
    case 0:
        return fs_.vendor;
    case 1:
        return fs_.product;
    case 2:
        return fs_.revision;
    default:
        return fs_.serial;
    }
}

// Next part after `part` that is not skipped; 0 when there is none.
std::uint8_t LssMaster::nextPart(std::uint8_t part) const noexcept {
    for (std::uint8_t p = static_cast<std::uint8_t>(part + 1); p <= 3; ++p) {
        if (mode(p) != LssScan::Skip) {
            return p;
        }
    }
    return 0;
}

void LssMaster::sendFastscan(std::uint32_t id, std::uint8_t bit, std::uint8_t sub, std::uint8_t next,
                             TimePoint now) noexcept {
    acked_ = false;
    queue(kFastscan, id, bit, sub, next);
    pump(now);
}

void LssMaster::beginPart(TimePoint now) noexcept {
    if (mode(part_) == LssScan::Match) {
        const std::uint32_t values[4] = {fs_.match.vendorId, fs_.match.productCode, fs_.match.revision,
                                         fs_.match.serial};
        id_ = values[part_];
        beginVerify(now);
        return;
    }
    id_ = 0;
    bit_ = 31;
    phase_ = Phase::FastscanScan;
    sendFastscan(id_, bit_, part_, part_, now);
}

void LssMaster::beginVerify(TimePoint now) noexcept {
    phase_ = Phase::FastscanVerify;
    sendFastscan(id_, 0, part_, nextPart(part_), now);
}

void LssMaster::fastscanStepDone(TimePoint now) noexcept {
    switch (phase_) {
    case Phase::FastscanCheck:
        if (!acked_) {
            finish(LssStatus::NoNode);
            return;
        }
        part_ = 0;
        beginPart(now);
        break;
    case Phase::FastscanScan:
        if (!acked_) {
            id_ |= std::uint32_t{1} << bit_;  // nobody has a 0 here
        }
        if (bit_ == 0) {
            beginVerify(now);
        } else {
            --bit_;
            sendFastscan(id_, bit_, part_, part_, now);
        }
        break;
    case Phase::FastscanVerify: {
        if (!acked_) {
            finish(LssStatus::NoAck);
            return;
        }
        std::uint32_t* found[4] = {&result_.address.vendorId, &result_.address.productCode,
                                   &result_.address.revision, &result_.address.serial};
        *found[part_] = id_;
        result_.knownParts = static_cast<std::uint8_t>(result_.knownParts | (1u << part_));
        const std::uint8_t next = nextPart(part_);
        if (next == 0) {
            selection_ = Selection::Selective;  // the slave is in configuration state now
            finish(LssStatus::Ok);
            return;
        }
        part_ = next;
        beginPart(now);
        break;
    }
    case Phase::Idle:
    case Phase::Response:
        break;
    }
}

} // namespace canopen_conductor
