#include "slcan_bus.hpp"

#include <cstring>

namespace canopen_conductor::slcan {

bool SlcanBus::open(const char* device, unsigned serialBaud, unsigned canKbit) noexcept {
    close();
    const char* rate = bitrateCommand(canKbit);
    if (rate == nullptr || !port_.open(device, serialBaud)) {
        return false;
    }
    lost_ = false;
    txHead_ = txCount_ = 0;
    rxHead_ = rxCount_ = 0;
    status_ = BusStatus{};
    changed_ = false;
    // Close first (the channel may still be open from a previous run), then
    // set the bit rate and open. Errors (BEL) for "C" on a closed channel
    // are harmless and skipped like every other reply.
    queue("C\r", 2);
    queue(rate, std::strlen(rate));
    queue("\r", 1);
    queue("O\r", 2);
    return flush();
}

void SlcanBus::close() noexcept {
    if (port_.isOpen()) {
        port_.write("C\r", 2);
        port_.close();
    }
}

bool SlcanBus::queue(const char* text, std::size_t n) noexcept {
    if (n > sizeof tx_ - txCount_) {
        return false;
    }
    for (std::size_t i = 0; i < n; ++i) {
        tx_[(txHead_ + txCount_ + i) % sizeof tx_] = text[i];
    }
    txCount_ += n;
    return true;
}

// Hand queued bytes to the OS without waiting. False if the port failed.
bool SlcanBus::flush() noexcept {
    while (txCount_ > 0) {
        // Contiguous part of the ring.
        const std::size_t chunk = txHead_ + txCount_ <= sizeof tx_ ? txCount_ : sizeof tx_ - txHead_;
        const long n = port_.write(tx_ + txHead_, chunk);
        if (n < 0) {
            lost_ = true;
            return false;
        }
        if (n == 0) {
            return true;  // the OS buffer is full; later
        }
        txHead_ = (txHead_ + static_cast<std::size_t>(n)) % sizeof tx_;
        txCount_ -= static_cast<std::size_t>(n);
    }
    return true;
}

bool SlcanBus::send(const CanFrame& frame) noexcept {
    if (lost_ || !port_.isOpen()) {
        return false;
    }
    char text[kMaxLine];
    const std::size_t n = encode(frame, text);
    if (n == 0 || !queue(text, n)) {
        return false;
    }
    flush();
    return true;
}

bool SlcanBus::pollStatus() noexcept {
    const bool ok = queue("F\r", 2);
    flush();
    return ok;
}

BusStatus SlcanBus::takeBusStatus() noexcept {
    const BusStatus s = status_;
    status_.rxOverflow = false;
    status_.txOverflow = false;
    changed_ = false;
    return s;
}

void SlcanBus::onLine(const char* line, std::size_t n) noexcept {
    CanFrame f;
    if (decode(line, n, f)) {
        if (rxCount_ == sizeof rx_ / sizeof rx_[0]) {
            status_.rxOverflow = true;  // host too slow: drop the frame
            changed_ = true;
            return;
        }
        rx_[(rxHead_ + rxCount_) % (sizeof rx_ / sizeof rx_[0])] = f;
        ++rxCount_;
        return;
    }
    BusStatus s;
    if (decodeStatus(line, n, s)) {
        if (s.state != status_.state || s.rxOverflow || s.txOverflow) {
            changed_ = true;
        }
        status_.state = s.state;
        status_.rxOverflow = status_.rxOverflow || s.rxOverflow;
        status_.txOverflow = status_.txOverflow || s.txOverflow;
    }
    // Anything else (OK, error, z/Z acknowledgements, version replies): skipped.
}

RxStatus SlcanBus::receive(CanFrame& out, Duration timeout) noexcept {
    if (lost_ || !port_.isOpen()) {
        return RxStatus::Error;
    }
    if (!flush()) {
        return RxStatus::Error;
    }
    if (rxCount_ == 0) {
        char buf[256];
        const long n = port_.read(buf, sizeof buf, timeout);
        if (n < 0) {
            lost_ = true;
            return RxStatus::Error;
        }
        for (long i = 0; i < n; ++i) {
            if (lines_.push(buf[i])) {
                onLine(lines_.line(), lines_.length());
            }
        }
    }
    if (rxCount_ == 0) {
        return RxStatus::Timeout;
    }
    out = rx_[rxHead_];
    rxHead_ = (rxHead_ + 1) % (sizeof rx_ / sizeof rx_[0]);
    --rxCount_;
    return RxStatus::Frame;
}

} // namespace canopen_conductor::slcan
