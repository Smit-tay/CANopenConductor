#include "canopen_conductor/sdo_client.hpp"

#include "crc16.hpp"

#include <cstring>

namespace canopen_conductor {

namespace {

constexpr std::size_t kSegmentBytes = 7;

std::uint16_t getU16(const std::uint8_t* p) noexcept {
    return static_cast<std::uint16_t>(p[0] | (p[1] << 8));
}

std::uint32_t getU32(const std::uint8_t* p) noexcept {
    return static_cast<std::uint32_t>(p[0]) | (static_cast<std::uint32_t>(p[1]) << 8) |
           (static_cast<std::uint32_t>(p[2]) << 16) | (static_cast<std::uint32_t>(p[3]) << 24);
}

void putU32(std::uint8_t* p, std::uint32_t v) noexcept {
    p[0] = static_cast<std::uint8_t>(v);
    p[1] = static_cast<std::uint8_t>(v >> 8);
    p[2] = static_cast<std::uint8_t>(v >> 16);
    p[3] = static_cast<std::uint8_t>(v >> 24);
}

std::size_t smaller(std::size_t a, std::size_t b) noexcept {
    return a < b ? a : b;
}

// An 8-byte frame with byte 0 = command and, if given, the object in bytes 1..3.
CanFrame request(std::uint32_t id, std::uint8_t command) noexcept {
    CanFrame f;
    f.id = id;
    f.len = 8;
    f.data[0] = command;
    return f;
}

CanFrame request(std::uint32_t id, std::uint8_t command, std::uint16_t index, std::uint8_t subIndex) noexcept {
    CanFrame f = request(id, command);
    f.data[1] = static_cast<std::uint8_t>(index);
    f.data[2] = static_cast<std::uint8_t>(index >> 8);
    f.data[3] = subIndex;
    return f;
}

} // namespace

SdoClient::SdoClient(ICanBus& bus, const SdoClientConfig& config) noexcept : bus_(&bus), config_(config) {}

// ---------------------------------------------------------------------------
// Requests

SdoStart SdoClient::upload(const SdoUpload& r, TimePoint now) noexcept {
    if (busy()) {
        return SdoStart::Busy;
    }
    if ((r.buffer == nullptr && r.capacity > 0) || r.timeout <= Duration::zero() ||
        (r.block && (config_.blockSize < 1 || config_.blockSize > 127))) {
        return SdoStart::InvalidArgument;
    }
    index_ = r.index;
    subIndex_ = r.subIndex;
    upBuffer_ = r.buffer;
    capacity_ = r.capacity;
    downData_ = nullptr;
    size_ = 0;
    sizeKnown_ = false;
    done_ = 0;
    timeout_ = r.timeout;
    lastHeard_ = now;
    toggle_ = 0;
    lastSent_ = false;
    seqno_ = 0;
    crcOn_ = false;
    crc_ = 0;

    if (r.block) {
        CanFrame f = request(config_.requestId, 0xA4, index_, subIndex_);
        f.data[4] = config_.blockSize;
        f.data[5] = config_.switchThreshold;
        state_ = State::UploadBlockInitiate;
        send(f);
    } else {
        state_ = State::UploadInitiate;
        send(request(config_.requestId, 0x40, index_, subIndex_));
    }
    return SdoStart::Started;
}

SdoStart SdoClient::download(const SdoDownload& r, TimePoint now) noexcept {
    if (busy()) {
        return SdoStart::Busy;
    }
    if ((r.data == nullptr && r.size > 0) || r.timeout <= Duration::zero() ||
        static_cast<std::uint64_t>(r.size) > 0xFFFFFFFFu ||
        (r.block && (config_.maxBurst < 1 || config_.maxBurst > 127))) {
        return SdoStart::InvalidArgument;
    }
    index_ = r.index;
    subIndex_ = r.subIndex;
    upBuffer_ = nullptr;
    capacity_ = 0;
    downData_ = r.data;
    size_ = r.size;
    sizeKnown_ = true;
    done_ = 0;
    timeout_ = r.timeout;
    lastHeard_ = now;
    toggle_ = 0;
    lastSent_ = false;
    seqno_ = 0;
    crcOn_ = false;
    crc_ = 0;

    CanFrame f;
    if (r.block) {
        f = request(config_.requestId, 0xC6, index_, subIndex_);
        putU32(&f.data[4], static_cast<std::uint32_t>(size_));
        state_ = State::DownloadBlockInitiate;
    } else if (size_ >= 1 && size_ <= 4) {
        const auto unused = static_cast<std::uint8_t>(4 - size_);
        f = request(config_.requestId, static_cast<std::uint8_t>(0x23 | (unused << 2)), index_, subIndex_);
        std::memcpy(&f.data[4], downData_, size_);
        state_ = State::DownloadInitiate;
    } else {
        f = request(config_.requestId, 0x21, index_, subIndex_);
        putU32(&f.data[4], static_cast<std::uint32_t>(size_));
        state_ = State::DownloadInitiate;
    }
    send(f);
    return SdoStart::Started;
}

void SdoClient::cancel(std::uint32_t abortCode) noexcept {
    if (busy()) {
        clientAbort(abortCode);
    }
}

// ---------------------------------------------------------------------------
// Event entry points

void SdoClient::onFrame(const CanFrame& frame, TimePoint now) noexcept {
    if (!busy() || frame.id != config_.responseId || !isUsable(frame) || frame.len != 8) {
        return;
    }
    const std::uint8_t* d = frame.data;
    lastHeard_ = now;

    if (d[0] == 0x80) {
        outPending_ = false;  // whatever we were about to send is moot
        finish(SdoStatus::ServerAbort, getU32(&d[4]));
        return;
    }

    switch (state_) {
    case State::UploadInitiate:
        onUploadInitiateResponse(d);
        break;
    case State::UploadSegment:
        onUploadSegment(d);
        break;
    case State::UploadBlockInitiate:
        onUploadBlockInitiateResponse(d, now);
        break;
    case State::UploadBlockSegments:
        onUploadBlockSegment(d, now);
        break;
    case State::UploadBlockEnd:
        onUploadBlockEnd(d);
        break;
    case State::DownloadInitiate:
        onDownloadInitiateResponse(d);
        break;
    case State::DownloadSegment:
        onDownloadSegmentResponse(d);
        break;
    case State::DownloadBlockInitiate:
        onDownloadBlockInitiateResponse(d);
        break;
    case State::DownloadBlockSending:
    case State::DownloadBlockAck:
        onDownloadBlockAck(d);
        break;
    case State::DownloadBlockEnd:
        if ((d[0] & 0xE3) != 0xA1) {
            clientAbort(sdo_abort::Command);
            return;
        }
        done_ = size_;
        finish(SdoStatus::Ok, 0);
        break;
    case State::Idle:
        break;
    }
}

TimePoint SdoClient::nextDeadline(TimePoint now) const noexcept {
    if (outPending_ || state_ == State::DownloadBlockSending) {
        return now;  // frames waiting for the send queue, or more segments of the sub-block
    }
    if (!busy()) {
        return TimePoint::max();
    }
    TimePoint d = lastHeard_ + timeout_;
    if (state_ == State::UploadBlockSegments) {
        const TimePoint gap = lastSegment_ + timeout_ * 7 / 10;
        d = gap < d ? gap : d;
    }
    return d;
}

void SdoClient::tick(TimePoint now) noexcept {
    pump();
    if (!busy()) {
        return;
    }
    if (now - lastHeard_ >= timeout_) {
        clientAbort(sdo_abort::Timeout);
        return;
    }
    // Server went quiet inside a sub-block: ack what we have so it can resend.
    if (state_ == State::UploadBlockSegments && now - lastSegment_ >= timeout_ * 7 / 10) {
        sendUploadBlockAck(now);
    }
}

// ---------------------------------------------------------------------------
// Sending and finishing

void SdoClient::send(const CanFrame& frame) noexcept {
    out_ = frame;
    outPending_ = true;
    pump();
}

// Push the waiting frame, then block-download segments: at most maxBurst
// frames per call, fewer if the send queue fills up.
void SdoClient::pump() noexcept {
    unsigned sent = 0;
    if (outPending_) {
        if (!bus_->send(out_)) {
            return;
        }
        outPending_ = false;
        ++sent;
    }
    while (state_ == State::DownloadBlockSending) {
        if (lastSent_ || seqno_ >= blockSize_) {
            state_ = State::DownloadBlockAck;
            return;
        }
        if (sent >= config_.maxBurst) {
            return;  // rest on the next tick
        }
        const std::size_t start = blockStart_ + std::size_t{seqno_} * kSegmentBytes;
        const std::size_t count = smaller(kSegmentBytes, size_ - start);
        const bool last = start + count == size_;
        ++seqno_;
        out_ = request(config_.requestId, static_cast<std::uint8_t>((last ? 0x80 : 0x00) | seqno_));
        if (count > 0) {
            std::memcpy(&out_.data[1], downData_ + start, count);
        }
        lastSent_ = last;
        if (!bus_->send(out_)) {
            outPending_ = true;
            return;
        }
        ++sent;
    }
}

void SdoClient::finish(SdoStatus status, std::uint32_t abortCode) noexcept {
    SdoResult result;
    result.status = status;
    result.abortCode = abortCode;
    result.nodeId = config_.nodeId;
    result.index = index_;
    result.subIndex = subIndex_;
    result.bytes = done_;
    state_ = State::Idle;
    if (config_.onDone != nullptr) {
        config_.onDone(config_.context, result);
    }
}

void SdoClient::clientAbort(std::uint32_t abortCode) noexcept {
    CanFrame f = request(config_.requestId, 0x80, index_, subIndex_);
    putU32(&f.data[4], abortCode);
    state_ = State::Idle;  // stop block-download streaming before sending
    send(f);
    finish(SdoStatus::ClientAbort, abortCode);
}

bool SdoClient::sameObject(const std::uint8_t* d) const noexcept {
    return getU16(&d[1]) == index_ && d[3] == subIndex_;
}

// ---------------------------------------------------------------------------
// Upload

void SdoClient::onUploadInitiateResponse(const std::uint8_t* d) noexcept {
    if ((d[0] & 0xE0) != 0x40) {
        clientAbort(sdo_abort::Command);
        return;
    }
    if (!sameObject(d)) {
        clientAbort(sdo_abort::ParameterIncompatible);
        return;
    }
    const bool expedited = (d[0] & 0x02) != 0;
    const bool sized = (d[0] & 0x01) != 0;

    if (expedited) {
        const std::size_t count = sized ? 4u - ((d[0] >> 2) & 0x03u) : 4u;
        if (count > capacity_) {
            // The server has already finished; nothing to abort on the bus.
            finish(SdoStatus::ClientAbort, sdo_abort::OutOfMemory);
            return;
        }
        std::memcpy(upBuffer_, &d[4], count);
        done_ = count;
        finish(SdoStatus::Ok, 0);
        return;
    }

    if (sized) {
        size_ = getU32(&d[4]);
        sizeKnown_ = true;
        if (size_ > capacity_) {
            clientAbort(sdo_abort::OutOfMemory);
            return;
        }
    }
    toggle_ = 0;
    state_ = State::UploadSegment;
    send(request(config_.requestId, 0x60));
}

void SdoClient::onUploadSegment(const std::uint8_t* d) noexcept {
    if ((d[0] & 0xE0) != 0x00) {
        clientAbort(sdo_abort::Command);
        return;
    }
    if (((d[0] >> 4) & 0x01) != toggle_) {
        clientAbort(sdo_abort::ToggleBit);
        return;
    }
    const std::size_t count = kSegmentBytes - ((d[0] >> 1) & 0x07u);
    const bool last = (d[0] & 0x01) != 0;

    if (sizeKnown_ && done_ + count > size_) {
        clientAbort(sdo_abort::TooLong);
        return;
    }
    if (done_ + count > capacity_) {
        clientAbort(sdo_abort::OutOfMemory);
        return;
    }
    if (count > 0) {
        std::memcpy(upBuffer_ + done_, &d[1], count);
    }
    done_ += count;

    if (last) {
        if (sizeKnown_ && done_ != size_) {
            clientAbort(sdo_abort::TooShort);
            return;
        }
        finish(SdoStatus::Ok, 0);
        return;
    }
    toggle_ ^= 1;
    send(request(config_.requestId, static_cast<std::uint8_t>(0x60 | (toggle_ << 4))));
}

void SdoClient::onUploadBlockInitiateResponse(const std::uint8_t* d, TimePoint now) noexcept {
    if ((d[0] & 0xE0) == 0x40) {
        // Server chose a normal upload instead (size <= pst).
        state_ = State::UploadInitiate;
        onUploadInitiateResponse(d);
        return;
    }
    if ((d[0] & 0xE1) != 0xC0) {
        clientAbort(sdo_abort::Command);
        return;
    }
    if (!sameObject(d)) {
        clientAbort(sdo_abort::ParameterIncompatible);
        return;
    }
    crcOn_ = (d[0] & 0x04) != 0;
    if ((d[0] & 0x02) != 0) {
        size_ = getU32(&d[4]);
        sizeKnown_ = true;
        if (size_ > capacity_) {
            clientAbort(sdo_abort::OutOfMemory);
            return;
        }
    }
    blockSize_ = config_.blockSize;
    seqno_ = 0;
    lastSent_ = false;
    lastSegment_ = now;
    state_ = State::UploadBlockSegments;
    send(request(config_.requestId, 0xA3));
}

void SdoClient::onUploadBlockSegment(const std::uint8_t* d, TimePoint now) noexcept {
    const auto seq = static_cast<std::uint8_t>(d[0] & 0x7F);
    const bool last = (d[0] & 0x80) != 0;

    if (seq == seqno_ + 1 && seq <= blockSize_) {
        if (!last) {
            if (sizeKnown_ && done_ + kSegmentBytes > size_) {
                clientAbort(sdo_abort::TooLong);
                return;
            }
            if (done_ + kSegmentBytes > capacity_) {
                clientAbort(sdo_abort::OutOfMemory);
                return;
            }
            std::memcpy(upBuffer_ + done_, &d[1], kSegmentBytes);
            if (crcOn_) {
                crc_ = detail::crc16(crc_, &d[1], kSegmentBytes);
            }
        } else {
            // Padding is unknown until the end frame; keep what fits, check then.
            const std::size_t room = smaller(kSegmentBytes, capacity_ - done_);
            if (room > 0) {
                std::memcpy(upBuffer_ + done_, &d[1], room);
            }
        }
        done_ += kSegmentBytes;
        seqno_ = seq;
        lastSegment_ = now;
        if (last) {
            lastSent_ = true;
            sendUploadBlockAck(now);
        } else if (seq == blockSize_) {
            sendUploadBlockAck(now);
        }
        return;
    }
    if (seq == seqno_ || seqno_ == 0) {
        return;  // repeat, or leftovers before the sub-block started
    }
    sendUploadBlockAck(now);  // out of order: ack the good part, server resends the rest
}

void SdoClient::sendUploadBlockAck(TimePoint now) noexcept {
    CanFrame f = request(config_.requestId, 0xA2);
    f.data[1] = seqno_;
    f.data[2] = config_.blockSize;
    blockSize_ = config_.blockSize;
    if (lastSent_) {
        state_ = State::UploadBlockEnd;
    } else {
        seqno_ = 0;
    }
    lastSegment_ = now;
    send(f);
}

void SdoClient::onUploadBlockEnd(const std::uint8_t* d) noexcept {
    if ((d[0] & 0xE3) != 0xC1) {
        clientAbort(sdo_abort::Command);
        return;
    }
    const std::size_t unused = (d[0] >> 2) & 0x07u;
    const std::size_t lastStart = done_ - kSegmentBytes;  // done_ includes the whole last segment
    const std::size_t valid = done_ - unused;
    if (valid > capacity_) {
        clientAbort(sdo_abort::OutOfMemory);
        return;
    }
    if (sizeKnown_ && valid != size_) {
        clientAbort(valid > size_ ? sdo_abort::TooLong : sdo_abort::TooShort);
        return;
    }
    if (crcOn_) {
        if (valid > lastStart) {
            crc_ = detail::crc16(crc_, upBuffer_ + lastStart, valid - lastStart);
        }
        if (crc_ != getU16(&d[1])) {
            clientAbort(sdo_abort::Crc);
            return;
        }
    }
    done_ = valid;
    send(request(config_.requestId, 0xA1));
    finish(SdoStatus::Ok, 0);
}

// ---------------------------------------------------------------------------
// Download

void SdoClient::onDownloadInitiateResponse(const std::uint8_t* d) noexcept {
    if ((d[0] & 0xE0) != 0x60) {
        clientAbort(sdo_abort::Command);
        return;
    }
    if (!sameObject(d)) {
        clientAbort(sdo_abort::ParameterIncompatible);
        return;
    }
    if (size_ >= 1 && size_ <= 4) {  // was expedited
        done_ = size_;
        finish(SdoStatus::Ok, 0);
        return;
    }
    toggle_ = 0;
    state_ = State::DownloadSegment;
    sendDownloadSegment();
}

void SdoClient::sendDownloadSegment() noexcept {
    const std::size_t count = smaller(kSegmentBytes, size_ - done_);
    const bool last = done_ + count == size_;
    const auto command = static_cast<std::uint8_t>((std::size_t{toggle_} << 4) | ((kSegmentBytes - count) << 1) | (last ? 1u : 0u));
    CanFrame f = request(config_.requestId, command);
    if (count > 0) {
        std::memcpy(&f.data[1], downData_ + done_, count);
    }
    segmentBytes_ = count;
    lastSent_ = last;
    send(f);
}

void SdoClient::onDownloadSegmentResponse(const std::uint8_t* d) noexcept {
    if ((d[0] & 0xE0) != 0x20) {
        clientAbort(sdo_abort::Command);
        return;
    }
    if (((d[0] >> 4) & 0x01) != toggle_) {
        clientAbort(sdo_abort::ToggleBit);
        return;
    }
    done_ += segmentBytes_;
    if (lastSent_) {
        finish(SdoStatus::Ok, 0);
        return;
    }
    toggle_ ^= 1;
    sendDownloadSegment();
}

void SdoClient::onDownloadBlockInitiateResponse(const std::uint8_t* d) noexcept {
    if ((d[0] & 0xE3) != 0xA0) {
        clientAbort(sdo_abort::Command);
        return;
    }
    if (!sameObject(d)) {
        clientAbort(sdo_abort::ParameterIncompatible);
        return;
    }
    if (d[4] < 1 || d[4] > 127) {
        clientAbort(sdo_abort::BlockSize);
        return;
    }
    crcOn_ = (d[0] & 0x04) != 0;
    crc_ = 0;
    blockSize_ = d[4];
    blockStart_ = 0;
    seqno_ = 0;
    lastSent_ = false;
    state_ = State::DownloadBlockSending;
    pump();
}

void SdoClient::onDownloadBlockAck(const std::uint8_t* d) noexcept {
    if ((d[0] & 0xE3) != 0xA2) {
        clientAbort(sdo_abort::Command);
        return;
    }
    const std::uint8_t ackseq = d[1];
    if (ackseq > seqno_) {
        clientAbort(sdo_abort::SequenceNumber);
        return;
    }
    outPending_ = false;  // an unsent segment of this sub-block is no longer wanted

    const std::size_t confirmed = smaller(std::size_t{ackseq} * kSegmentBytes, size_ - blockStart_);
    if (crcOn_ && confirmed > 0) {
        crc_ = detail::crc16(crc_, downData_ + blockStart_, confirmed);
    }
    done_ = blockStart_ + confirmed;

    if (lastSent_ && ackseq == seqno_) {
        // Everything confirmed: send end with the unused bytes of the last segment.
        const std::size_t unused = size_ == 0 ? kSegmentBytes : (kSegmentBytes - size_ % kSegmentBytes) % kSegmentBytes;
        CanFrame f = request(config_.requestId, static_cast<std::uint8_t>(0xC1 | (unused << 2)));
        const std::uint16_t crc = crcOn_ ? crc_ : 0;
        f.data[1] = static_cast<std::uint8_t>(crc);
        f.data[2] = static_cast<std::uint8_t>(crc >> 8);
        state_ = State::DownloadBlockEnd;
        send(f);
        return;
    }
    if (d[2] < 1 || d[2] > 127) {
        clientAbort(sdo_abort::BlockSize);
        return;
    }
    blockSize_ = d[2];
    blockStart_ = done_;
    seqno_ = 0;
    lastSent_ = false;
    state_ = State::DownloadBlockSending;
    pump();
}

} // namespace canopen_conductor
