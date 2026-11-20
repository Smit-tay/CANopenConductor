#include "canopen_conductor/socketcan_bus.hpp"

#include <cerrno>
#include <cstring>
#include <linux/can.h>
#include <linux/can/error.h>
#include <linux/can/netlink.h>
#include <linux/can/raw.h>
#include <linux/if_link.h>
#include <linux/netlink.h>
#include <linux/rtnetlink.h>
#include <net/if.h>
#include <poll.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <sys/uio.h>
#include <time.h>
#include <unistd.h>

namespace canopen_conductor::linux_port {

namespace {

// Netlink attributes are a type and a length, then the payload, padded to 4 bytes. A nested attribute is a list of
// attributes in its payload.
constexpr std::size_t align4(std::size_t n) noexcept {
    return (n + 3U) & ~std::size_t{3};
}

struct Span {
    const unsigned char* data = nullptr;
    std::size_t size = 0;
};

// The payload of the first attribute of `type` in the attribute list `list`. The flag bits the kernel sets in the
// type of a nested attribute are ignored.
bool findAttribute(Span list, unsigned type, Span& payload) noexcept {
    constexpr unsigned kTypeMask = 0x3FFFU;
    while (list.size >= sizeof(rtattr)) {
        rtattr header;
        std::memcpy(&header, list.data, sizeof header);
        if (header.rta_len < sizeof header || header.rta_len > list.size) {
            return false;
        }
        if ((static_cast<unsigned>(header.rta_type) & kTypeMask) == type) {
            payload = { list.data + sizeof header, header.rta_len - sizeof header };
            return true;
        }
        const std::size_t step = align4(header.rta_len);
        if (step >= list.size) {
            return false;  // that was the last one
        }
        list = { list.data + step, list.size - step };
    }
    return false;
}

} // namespace

ListenOnly SocketCanBus::parseListenOnly(const void* message, std::size_t size) noexcept {
    const auto* bytes = static_cast<const unsigned char*>(message);
    nlmsghdr header;
    if (message == nullptr || size < sizeof header) {
        return ListenOnly::Unknown;
    }
    std::memcpy(&header, bytes, sizeof header);
    const std::size_t attributesAt = sizeof header + align4(sizeof(ifinfomsg));
    if (header.nlmsg_type != RTM_NEWLINK || header.nlmsg_len < attributesAt || header.nlmsg_len > size) {
        return ListenOnly::Unknown;  // an error answer (NLMSG_ERROR), or not what was asked for
    }

    // IFLA_LINKINFO > IFLA_INFO_DATA > IFLA_CAN_CTRLMODE, which holds the control mode the interface is in
    Span linkInfo;
    Span data;
    Span mode;
    if (!findAttribute({ bytes + attributesAt, header.nlmsg_len - attributesAt }, IFLA_LINKINFO, linkInfo) ||
        !findAttribute(linkInfo, IFLA_INFO_DATA, data) || !findAttribute(data, IFLA_CAN_CTRLMODE, mode) ||
        mode.size < sizeof(can_ctrlmode)) {
        return ListenOnly::Unknown;
    }
    can_ctrlmode control;
    std::memcpy(&control, mode.data, sizeof control);
    return (control.flags & CAN_CTRLMODE_LISTENONLY) != 0 ? ListenOnly::Yes : ListenOnly::No;
}

ListenOnly SocketCanBus::queryListenOnly(const char* ifname) noexcept {
    const unsigned index = if_nametoindex(ifname);
    if (index == 0) {
        return ListenOnly::Unknown;
    }
    const int fd = ::socket(AF_NETLINK, SOCK_RAW | SOCK_CLOEXEC, NETLINK_ROUTE);
    if (fd < 0) {
        return ListenOnly::Unknown;
    }
    // The kernel answers at once. This only keeps a broken system from holding the I/O thread.
    const timeval limit { 0, 200'000 };
    ::setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &limit, sizeof limit);

    struct {
        nlmsghdr header;
        ifinfomsg info;
    } request {};
    request.header.nlmsg_len = sizeof request;
    request.header.nlmsg_type = RTM_GETLINK;
    request.header.nlmsg_flags = NLM_F_REQUEST;
    request.header.nlmsg_seq = 1;
    request.info.ifi_family = AF_UNSPEC;
    request.info.ifi_index = static_cast<int>(index);

    ListenOnly result = ListenOnly::Unknown;
    if (::send(fd, &request, sizeof request, 0) == static_cast<ssize_t>(sizeof request)) {
        alignas(nlmsghdr) unsigned char reply[16384];
        const ssize_t n = ::recv(fd, reply, sizeof reply, 0);
        if (n > 0) {
            result = parseListenOnly(reply, static_cast<std::size_t>(n));
        }
    }
    ::close(fd);
    return result;
}

SocketCanBus::~SocketCanBus() {
    close();
}

bool SocketCanBus::open(const char* ifname) noexcept {
    close();
    osError_ = 0;
    const unsigned index = if_nametoindex(ifname);
    if (index == 0) {
        osError_ = errno != 0 ? errno : ENODEV;
        return false;
    }
    fd_ = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (fd_ < 0) {
        osError_ = errno;
        return false;
    }
    // Controller problems, bus-off, missing ACK and bus errors as error frames.
    const can_err_mask_t errors = CAN_ERR_CRTL | CAN_ERR_BUSOFF | CAN_ERR_ACK | CAN_ERR_BUSERROR;
    ::setsockopt(fd_, SOL_CAN_RAW, CAN_RAW_ERR_FILTER, &errors, sizeof errors);
    const int on = 1;
    ::setsockopt(fd_, SOL_SOCKET, SO_RXQ_OVFL, &on, sizeof on);

    sockaddr_can addr{};
    addr.can_family = AF_CAN;
    addr.can_ifindex = static_cast<int>(index);
    if (::bind(fd_, reinterpret_cast<sockaddr*>(&addr), sizeof addr) < 0) {
        const int error = errno;
        close();
        osError_ = error;
        return false;
    }
    lost_ = false;
    changed_ = false;
    status_ = BusStatus{};
    dropped_ = 0;
    listenOnly_ = queryListenOnly(ifname);
    return true;
}

bool SocketCanBus::close() noexcept {
    if (fd_ < 0) {
        return true;
    }
    const bool ok = ::close(fd_) == 0;  // the descriptor is gone even when this fails; no retry
    if (!ok) {
        osError_ = errno;
    }
    fd_ = -1;
    return ok;
}

bool SocketCanBus::send(const CanFrame& frame) noexcept {
    if (fd_ < 0 || lost_ || frame.len > 8) {
        return false;
    }
    can_frame raw{};
    raw.can_id = frame.id;
    if (frame.extended) {
        raw.can_id |= CAN_EFF_FLAG;
    }
    if (frame.rtr) {
        raw.can_id |= CAN_RTR_FLAG;
    }
    raw.len = frame.len;
    std::memcpy(raw.data, frame.data, frame.len);
    for (;;) {
        const ssize_t n = ::send(fd_, &raw, sizeof raw, MSG_DONTWAIT);
        if (n == static_cast<ssize_t>(sizeof raw)) {
            return true;
        }
        if (n < 0 && errno == EINTR) {
            continue;
        }
        if (n < 0 && (errno == ENETDOWN || errno == ENODEV || errno == ENXIO)) {
            osError_ = errno;
            lost_ = true;  // reported by the next receive()
        }
        return false;  // EAGAIN / ENOBUFS: queue full
    }
}

void SocketCanBus::setState(BusState s) noexcept {
    if (status_.state != s) {
        status_.state = s;
        changed_ = true;
    }
}

void SocketCanBus::onErrorFrame(std::uint32_t canId, const std::uint8_t* data) noexcept {
    if ((canId & CAN_ERR_BUSOFF) != 0) {
        setState(BusState::BusOff);
        return;
    }
    if ((canId & CAN_ERR_CRTL) == 0) {
        return;
    }
    const std::uint8_t c = data[1];
    if ((c & (CAN_ERR_CRTL_RX_OVERFLOW)) != 0) {
        status_.rxOverflow = true;
        changed_ = true;
    }
    if ((c & (CAN_ERR_CRTL_TX_OVERFLOW)) != 0) {
        status_.txOverflow = true;
        changed_ = true;
    }
    if ((c & (CAN_ERR_CRTL_RX_PASSIVE | CAN_ERR_CRTL_TX_PASSIVE)) != 0) {
        setState(BusState::Passive);
    } else if ((c & (CAN_ERR_CRTL_RX_WARNING | CAN_ERR_CRTL_TX_WARNING | CAN_ERR_CRTL_ACTIVE)) != 0) {
        setState(BusState::Active);
    }
}

BusStatus SocketCanBus::takeBusStatus() noexcept {
    const BusStatus s = status_;
    status_.rxOverflow = false;
    status_.txOverflow = false;
    changed_ = false;
    return s;
}

RxStatus SocketCanBus::receive(CanFrame& out, Duration timeout) noexcept {
    if (fd_ < 0 || lost_) {
        if (osError_ == 0) {
            osError_ = ENETDOWN;
        }
        return RxStatus::Error;
    }
    pollfd p[2] = {};
    p[0].fd = fd_;
    p[0].events = POLLIN;
    nfds_t count = 1;
    if (waker_ != nullptr && waker_->valid()) {
        p[1].fd = waker_->fd();
        p[1].events = POLLIN;
        count = 2;
    }
    const auto us = timeout.count() > 0 ? timeout.count() : 0;
    timespec ts{};
    ts.tv_sec = static_cast<time_t>(us / 1'000'000);
    ts.tv_nsec = static_cast<long>((us % 1'000'000) * 1000);
    const int ready = ::ppoll(p, count, &ts, nullptr);
    if (ready == 0 || (ready < 0 && errno == EINTR)) {
        return RxStatus::Timeout;
    }
    if (ready < 0) {
        osError_ = errno;
        return RxStatus::Error;
    }
    if (count == 2 && p[1].revents != 0) {
        waker_->drain();  // woken. A frame that arrived at the same time is read below, not lost.
    }
    if ((p[0].revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
        // The interface went down or away. The socket's pending error says which, if it knows.
        int error = 0;
        socklen_t size = sizeof error;
        if (::getsockopt(fd_, SOL_SOCKET, SO_ERROR, &error, &size) != 0 || error == 0) {
            error = ENETDOWN;
        }
        osError_ = error;
        lost_ = true;
        return RxStatus::Error;
    }
    if ((p[0].revents & POLLIN) == 0) {
        return RxStatus::Timeout;  // only the wake
    }

    can_frame raw{};
    iovec iov{&raw, sizeof raw};
    alignas(cmsghdr) unsigned char control[CMSG_SPACE(sizeof(std::uint32_t))];
    msghdr msg{};
    msg.msg_iov = &iov;
    msg.msg_iovlen = 1;
    msg.msg_control = control;
    msg.msg_controllen = sizeof control;
    const ssize_t n = ::recvmsg(fd_, &msg, 0);
    if (n < 0) {
        if (errno == EAGAIN || errno == EINTR) {
            return RxStatus::Timeout;
        }
        osError_ = errno;
        if (errno == ENETDOWN || errno == ENODEV || errno == ENXIO) {
            lost_ = true;
        }
        return RxStatus::Error;
    }
    for (cmsghdr* c = CMSG_FIRSTHDR(&msg); c != nullptr; c = CMSG_NXTHDR(&msg, c)) {
        if (c->cmsg_level == SOL_SOCKET && c->cmsg_type == SO_RXQ_OVFL) {
            std::uint32_t dropped = 0;
            std::memcpy(&dropped, CMSG_DATA(c), sizeof dropped);
            if (dropped != dropped_) {
                dropped_ = dropped;
                status_.rxOverflow = true;
                changed_ = true;
            }
        }
    }
    if (n != static_cast<ssize_t>(sizeof raw)) {
        return RxStatus::Timeout;  // not a classic CAN frame; skip it
    }
    if ((raw.can_id & CAN_ERR_FLAG) != 0) {
        onErrorFrame(raw.can_id & CAN_ERR_MASK, raw.data);
        return RxStatus::Timeout;
    }
    if (status_.state == BusState::BusOff) {
        setState(BusState::Active);  // receiving again: the controller was restarted
    }
    out = CanFrame{};
    out.extended = (raw.can_id & CAN_EFF_FLAG) != 0;
    out.rtr = (raw.can_id & CAN_RTR_FLAG) != 0;
    out.id = raw.can_id & (out.extended ? CAN_EFF_MASK : CAN_SFF_MASK);
    out.len = raw.len > 8 ? 8 : raw.len;
    std::memcpy(out.data, raw.data, out.len);
    return RxStatus::Frame;
}

} // namespace canopen_conductor::linux_port
