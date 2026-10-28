#include "canopen_conductor/waker.hpp"

#if defined(_WIN32)
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#else
#include <cerrno>
#include <fcntl.h>
#include <poll.h>
#include <time.h>
#include <unistd.h>
#endif

namespace canopen_conductor {

#if defined(_WIN32)

Waker::Waker() noexcept {
    event_ = ::CreateEventA(nullptr, TRUE, FALSE, nullptr);
}

Waker::~Waker() {
    if (event_ != nullptr) {
        ::CloseHandle(static_cast<HANDLE>(event_));
    }
}

bool Waker::valid() const noexcept {
    return event_ != nullptr;
}

void Waker::wake() noexcept {
    ::SetEvent(static_cast<HANDLE>(event_));
}

void Waker::drain() noexcept {
    ::ResetEvent(static_cast<HANDLE>(event_));
}

bool Waker::wait(Duration timeout) noexcept {
    const auto us = timeout.count() > 0 ? timeout.count() : 0;
    const DWORD ms = static_cast<DWORD>((us + 999) / 1000);
    const bool woken = ::WaitForSingleObject(static_cast<HANDLE>(event_), ms) == WAIT_OBJECT_0;
    if (woken) {
        drain();
    }
    return woken;
}

#else

namespace {

bool setNonBlockingCloexec(int fd) noexcept {
    const int fl = ::fcntl(fd, F_GETFL);
    return fl >= 0 && ::fcntl(fd, F_SETFL, fl | O_NONBLOCK) == 0 && ::fcntl(fd, F_SETFD, FD_CLOEXEC) == 0;
}

} // namespace

// A pipe, not an eventfd: the same code works on every POSIX system.
Waker::Waker() noexcept {
    if (::pipe(fds_) != 0) {
        fds_[0] = fds_[1] = -1;
        return;
    }
    if (!setNonBlockingCloexec(fds_[0]) || !setNonBlockingCloexec(fds_[1])) {
        ::close(fds_[0]);
        ::close(fds_[1]);
        fds_[0] = fds_[1] = -1;
    }
}

Waker::~Waker() {
    if (fds_[0] >= 0) {
        ::close(fds_[0]);
        ::close(fds_[1]);
    }
}

bool Waker::valid() const noexcept {
    return fds_[0] >= 0;
}

void Waker::wake() noexcept {
    const char byte = 1;
    // A full pipe means a wake is pending already, so EAGAIN is fine.
    while (::write(fds_[1], &byte, 1) < 0 && errno == EINTR) {
    }
}

void Waker::drain() noexcept {
    char buf[32];
    while (::read(fds_[0], buf, sizeof buf) > 0) {
    }
}

bool Waker::wait(Duration timeout) noexcept {
    pollfd p{};
    p.fd = fds_[0];
    p.events = POLLIN;
    const auto us = timeout.count() > 0 ? timeout.count() : 0;
    timespec ts{};
    ts.tv_sec = static_cast<time_t>(us / 1'000'000);
    ts.tv_nsec = static_cast<long>((us % 1'000'000) * 1000);
    const int ready = ::ppoll(&p, 1, &ts, nullptr);
    if (ready > 0 && (p.revents & POLLIN) != 0) {
        drain();
        return true;
    }
    return false;
}

#endif

} // namespace canopen_conductor
