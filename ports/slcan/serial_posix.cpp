#include "serial_port.hpp"

#include <cerrno>
#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

namespace canopen_conductor::slcan {

namespace {

speed_t speedOf(unsigned baud) noexcept {
    switch (baud) {
    case 9600:
        return B9600;
    case 19200:
        return B19200;
    case 38400:
        return B38400;
    case 57600:
        return B57600;
    case 230400:
        return B230400;
    case 460800:
        return B460800;
    case 921600:
        return B921600;
    case 1000000:
        return B1000000;
    case 2000000:
        return B2000000;
    case 3000000:
        return B3000000;
    default:
        return B115200;
    }
}

} // namespace

SerialPort::~SerialPort() {
    close();
}

bool SerialPort::open(const char* device, unsigned baud) noexcept {
    close();
    fd_ = ::open(device, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
        return false;
    }
    termios t{};
    if (::tcgetattr(fd_, &t) == 0) {  // a pty or a USB CDC device also has termios
        ::cfmakeraw(&t);
        t.c_cflag |= CLOCAL | CREAD;
        t.c_cflag &= ~static_cast<tcflag_t>(CRTSCTS);
        ::cfsetispeed(&t, speedOf(baud));
        ::cfsetospeed(&t, speedOf(baud));
        ::tcsetattr(fd_, TCSANOW, &t);
    }
    return true;
}

void SerialPort::close() noexcept {
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
}

bool SerialPort::isOpen() const noexcept {
    return fd_ >= 0;
}

long SerialPort::write(const char* data, std::size_t size) noexcept {
    if (fd_ < 0) {
        return -1;
    }
    for (;;) {
        const ssize_t n = ::write(fd_, data, size);
        if (n >= 0) {
            return static_cast<long>(n);
        }
        if (errno == EINTR) {
            continue;
        }
        return errno == EAGAIN ? 0 : -1;
    }
}

long SerialPort::read(char* buf, std::size_t capacity, Duration timeout) noexcept {
    if (fd_ < 0) {
        return -1;
    }
    pollfd p{};
    p.fd = fd_;
    p.events = POLLIN;
    const auto us = timeout.count() > 0 ? timeout.count() : 0;
    timespec ts{};
    ts.tv_sec = static_cast<time_t>(us / 1'000'000);
    ts.tv_nsec = static_cast<long>((us % 1'000'000) * 1000);
    const int ready = ::ppoll(&p, 1, &ts, nullptr);
    if (ready == 0 || (ready < 0 && errno == EINTR)) {
        return 0;
    }
    if (ready < 0 || (p.revents & (POLLERR | POLLNVAL)) != 0) {
        return -1;
    }
    const ssize_t n = ::read(fd_, buf, capacity);
    if (n > 0) {
        return static_cast<long>(n);
    }
    if (n == 0 || (p.revents & POLLHUP) != 0) {
        return -1;  // device gone
    }
    return (errno == EAGAIN || errno == EINTR) ? 0 : -1;
}

} // namespace canopen_conductor::slcan
