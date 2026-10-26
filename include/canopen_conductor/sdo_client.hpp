#pragma once

#include "canopen_conductor/can_bus.hpp"

#include <cstddef>
#include <cstdint>

namespace canopen_conductor {

// SDO abort codes (CiA 301 table 22).
namespace sdo_abort {
inline constexpr std::uint32_t ToggleBit = 0x05030000;
inline constexpr std::uint32_t Timeout = 0x05040000;
inline constexpr std::uint32_t Command = 0x05040001;
inline constexpr std::uint32_t BlockSize = 0x05040002;
inline constexpr std::uint32_t SequenceNumber = 0x05040003;
inline constexpr std::uint32_t Crc = 0x05040004;
inline constexpr std::uint32_t OutOfMemory = 0x05040005;
inline constexpr std::uint32_t UnsupportedAccess = 0x06010000;
inline constexpr std::uint32_t WriteOnly = 0x06010001;
inline constexpr std::uint32_t ReadOnly = 0x06010002;
inline constexpr std::uint32_t NoObject = 0x06020000;
inline constexpr std::uint32_t NotMappable = 0x06040041;
inline constexpr std::uint32_t PdoLength = 0x06040042;
inline constexpr std::uint32_t ParameterIncompatible = 0x06040043;
inline constexpr std::uint32_t DeviceIncompatible = 0x06040047;
inline constexpr std::uint32_t Hardware = 0x06060000;
inline constexpr std::uint32_t LengthMismatch = 0x06070010;
inline constexpr std::uint32_t TooLong = 0x06070012;
inline constexpr std::uint32_t TooShort = 0x06070013;
inline constexpr std::uint32_t NoSubIndex = 0x06090011;
inline constexpr std::uint32_t InvalidValue = 0x06090030;
inline constexpr std::uint32_t ValueTooHigh = 0x06090031;
inline constexpr std::uint32_t ValueTooLow = 0x06090032;
inline constexpr std::uint32_t MaxBelowMin = 0x06090036;
inline constexpr std::uint32_t NoConnection = 0x060A0023;
inline constexpr std::uint32_t General = 0x08000000;
inline constexpr std::uint32_t CannotStore = 0x08000020;
inline constexpr std::uint32_t LocalControl = 0x08000021;
inline constexpr std::uint32_t DeviceState = 0x08000022;
inline constexpr std::uint32_t NoDictionary = 0x08000023;
inline constexpr std::uint32_t NoData = 0x08000024;
} // namespace sdo_abort

enum class SdoStatus {
    Ok,
    ServerAbort,  // server sent an abort; see abortCode
    ClientAbort,  // this client aborted; see abortCode
};

struct SdoResult {
    SdoStatus status = SdoStatus::Ok;
    std::uint32_t abortCode = 0;
    std::uint8_t nodeId = 0;  // from SdoClientConfig::nodeId
    std::uint16_t index = 0;
    std::uint8_t subIndex = 0;
    std::size_t bytes = 0;  // bytes transferred (upload: valid bytes in buffer)
};

// Called once per finished transfer, on the I/O thread. Must only signal.
using SdoDoneFn = void (*)(void* context, const SdoResult& result) noexcept;

struct SdoClientConfig {
    std::uint8_t nodeId = 0;             // only copied into SdoResult
    std::uint32_t requestId = 0;         // client to server, usually 0x600 + node
    std::uint32_t responseId = 0;        // server to client, usually 0x580 + node
    std::uint8_t blockSize = 127;        // segments per block we accept (block upload), 1..127
    std::uint8_t switchThreshold = 21;   // pst sent in block upload initiate
    std::uint8_t maxBurst = 8;           // frames sent per onFrame/tick call in block download, 1..127
    SdoDoneFn onDone = nullptr;
    void* context = nullptr;
};

// Read index:subIndex into buffer[0..capacity).
struct SdoUpload {
    std::uint16_t index = 0;
    std::uint8_t subIndex = 0;
    std::uint8_t* buffer = nullptr;
    std::size_t capacity = 0;
    Duration timeout = Duration{500'000};
    bool block = false;
};

// Write data[0..size) to index:subIndex.
struct SdoDownload {
    std::uint16_t index = 0;
    std::uint8_t subIndex = 0;
    const std::uint8_t* data = nullptr;
    std::size_t size = 0;
    Duration timeout = Duration{500'000};
    bool block = false;
};

enum class SdoStart {
    Started,
    Busy,             // a transfer is already running
    InvalidArgument,  // bad buffer, size, timeout, block size or burst limit
};

// One SDO channel to one server. The caller's buffer must stay valid until
// the done callback has fired.
class SdoClient {
public:
    SdoClient(ICanBus& bus, const SdoClientConfig& config) noexcept;

    SdoStart upload(const SdoUpload& request, TimePoint now) noexcept;
    SdoStart download(const SdoDownload& request, TimePoint now) noexcept;

    // Abort the running transfer (if any) with the given code.
    void cancel(std::uint32_t abortCode) noexcept;

    void onFrame(const CanFrame& frame, TimePoint now) noexcept;
    void tick(TimePoint now) noexcept;

    // Earliest time tick() has work to do if no frame arrives first:
    // `now` if a frame is waiting for room in the send queue, a past time if
    // something is already due, TimePoint::max() if nothing is timed.
    TimePoint nextDeadline(TimePoint now) const noexcept;

    bool busy() const noexcept { return state_ != State::Idle; }

private:
    enum class State : std::uint8_t {
        Idle,
        UploadInitiate,
        UploadSegment,
        UploadBlockInitiate,
        UploadBlockSegments,
        UploadBlockEnd,
        DownloadInitiate,
        DownloadSegment,
        DownloadBlockInitiate,
        DownloadBlockSending,
        DownloadBlockAck,
        DownloadBlockEnd,
    };

    void send(const CanFrame& frame) noexcept;
    void pump() noexcept;
    void finish(SdoStatus status, std::uint32_t abortCode) noexcept;
    void clientAbort(std::uint32_t abortCode) noexcept;
    bool sameObject(const std::uint8_t* d) const noexcept;

    void onUploadInitiateResponse(const std::uint8_t* d) noexcept;
    void onUploadSegment(const std::uint8_t* d) noexcept;
    void onUploadBlockInitiateResponse(const std::uint8_t* d, TimePoint now) noexcept;
    void onUploadBlockSegment(const std::uint8_t* d, TimePoint now) noexcept;
    void onUploadBlockEnd(const std::uint8_t* d) noexcept;
    void sendUploadBlockAck(TimePoint now) noexcept;

    void onDownloadInitiateResponse(const std::uint8_t* d) noexcept;
    void onDownloadSegmentResponse(const std::uint8_t* d) noexcept;
    void onDownloadBlockInitiateResponse(const std::uint8_t* d) noexcept;
    void onDownloadBlockAck(const std::uint8_t* d) noexcept;
    void sendDownloadSegment() noexcept;

    ICanBus* bus_;
    SdoClientConfig config_;
    State state_ = State::Idle;

    // Current transfer.
    std::uint16_t index_ = 0;
    std::uint8_t subIndex_ = 0;
    std::uint8_t* upBuffer_ = nullptr;
    std::size_t capacity_ = 0;
    const std::uint8_t* downData_ = nullptr;
    std::size_t size_ = 0;            // download: total size; upload: size given by server
    bool sizeKnown_ = false;          // upload: server gave a size
    std::size_t done_ = 0;            // bytes confirmed (download) or accepted (upload)
    Duration timeout_{};
    TimePoint lastHeard_{};           // last valid frame from server (or start)

    // Segmented transfer.
    std::uint8_t toggle_ = 0;
    std::size_t segmentBytes_ = 0;    // bytes in the download segment awaiting response
    bool lastSent_ = false;           // the segment with c=1 has been sent / accepted

    // Block transfer.
    std::uint8_t blockSize_ = 0;      // segments in the current sub-block
    std::uint8_t seqno_ = 0;          // last segment sent (download) or accepted (upload)
    std::size_t blockStart_ = 0;      // download: first byte of the current sub-block
    bool crcOn_ = false;
    std::uint16_t crc_ = 0;
    TimePoint lastSegment_{};         // upload: last accepted segment or ack sent

    // Frame waiting for room in the send queue.
    CanFrame out_{};
    bool outPending_ = false;
};

} // namespace canopen_conductor
