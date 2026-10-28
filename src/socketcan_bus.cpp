#include "canopen_conductor/socketcan_bus.hpp"

#include <cerrno>
#include <cstring>
#include <linux/can.h>
#include <linux/can/error.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <poll.h>
#include <sys/socket.h>
#include <sys/uio.h>
#include <time.h>
#include <unistd.h>

namespace canopen_conductor::linux_port {

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
