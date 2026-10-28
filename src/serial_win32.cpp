// Windows serial line for the slcan port. Overlapped I/O, so writes never
// wait and reads wait at most the given timeout.
// NOTE: not compiled or tested yet (no Windows toolchain on the
// development machine).

#include "canopen_conductor/serial_port.hpp"

#include <cstdio>
#include <cstring>

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>

namespace canopen_conductor::slcan {

static_assert(sizeof(OVERLAPPED) <= 64, "OVERLAPPED storage too small");

namespace {

OVERLAPPED* ov(unsigned char* storage) noexcept {
    return reinterpret_cast<OVERLAPPED*>(storage);
}

} // namespace

SerialPort::~SerialPort() {
    close();
}

bool SerialPort::open(const char* device, unsigned baud) noexcept {
    close();
    osError_ = 0;
    char path[64];
    std::snprintf(path, sizeof path, "\\\\.\\%s", device);  // also reaches COM10 and above
    HANDLE h = ::CreateFileA(path, GENERIC_READ | GENERIC_WRITE, 0, nullptr, OPEN_EXISTING, FILE_FLAG_OVERLAPPED,
                             nullptr);
    if (h == INVALID_HANDLE_VALUE) {
        osError_ = static_cast<int>(::GetLastError());
        return false;
    }
    DCB dcb{};
    dcb.DCBlength = sizeof dcb;
    ::GetCommState(h, &dcb);
    dcb.BaudRate = baud;
    dcb.ByteSize = 8;
    dcb.Parity = NOPARITY;
    dcb.StopBits = ONESTOPBIT;
    dcb.fBinary = TRUE;
    dcb.fOutxCtsFlow = FALSE;
    dcb.fOutxDsrFlow = FALSE;
    dcb.fDtrControl = DTR_CONTROL_ENABLE;
    dcb.fRtsControl = RTS_CONTROL_ENABLE;
    dcb.fOutX = FALSE;
    dcb.fInX = FALSE;
    if (!::SetCommState(h, &dcb)) {
        osError_ = static_cast<int>(::GetLastError());
        ::CloseHandle(h);
        return false;
    }
    handle_ = h;
    readEvent_ = ::CreateEventA(nullptr, TRUE, FALSE, nullptr);
    writeEvent_ = ::CreateEventA(nullptr, TRUE, FALSE, nullptr);
    std::memset(readOv_, 0, sizeof readOv_);
    std::memset(writeOv_, 0, sizeof writeOv_);
    writing_ = false;
    if (readEvent_ == nullptr || writeEvent_ == nullptr) {
        osError_ = static_cast<int>(::GetLastError());
        close();
        return false;
    }
    return true;
}

bool SerialPort::close() noexcept {
    bool ok = true;
    if (handle_ != nullptr) {
        ::CancelIo(static_cast<HANDLE>(handle_));
        if (!::CloseHandle(static_cast<HANDLE>(handle_))) {
            osError_ = static_cast<int>(::GetLastError());
            ok = false;
        }
        handle_ = nullptr;
    }
    if (readEvent_ != nullptr) {
        ::CloseHandle(static_cast<HANDLE>(readEvent_));
        readEvent_ = nullptr;
    }
    if (writeEvent_ != nullptr) {
        ::CloseHandle(static_cast<HANDLE>(writeEvent_));
        writeEvent_ = nullptr;
    }
    return ok;
}

bool SerialPort::isOpen() const noexcept {
    return handle_ != nullptr;
}

long SerialPort::write(const char* data, std::size_t size) noexcept {
    if (handle_ == nullptr) {
        return -1;
    }
    HANDLE h = static_cast<HANDLE>(handle_);
    if (writing_) {  // one overlapped write at a time
        DWORD done = 0;
        if (!::GetOverlappedResult(h, ov(writeOv_), &done, FALSE)) {
            if (::GetLastError() == ERROR_IO_INCOMPLETE) {
                return 0;
            }
            osError_ = static_cast<int>(::GetLastError());
            return -1;
        }
        writing_ = false;
    }
    const std::size_t n = size < sizeof writeBuf_ ? size : sizeof writeBuf_;
    std::memcpy(writeBuf_, data, n);
    std::memset(writeOv_, 0, sizeof writeOv_);
    ov(writeOv_)->hEvent = static_cast<HANDLE>(writeEvent_);
    if (!::WriteFile(h, writeBuf_, static_cast<DWORD>(n), nullptr, ov(writeOv_))) {
        if (::GetLastError() != ERROR_IO_PENDING) {
            osError_ = static_cast<int>(::GetLastError());
            return -1;
        }
        writing_ = true;  // completes in the background; the bytes are ours to forget
    }
    return static_cast<long>(n);
}

long SerialPort::read(char* buf, std::size_t capacity, Duration timeout) noexcept {
    if (handle_ == nullptr) {
        return -1;
    }
    HANDLE h = static_cast<HANDLE>(handle_);
    const auto ms = timeout.count() <= 0 ? 0 : (timeout.count() + 999) / 1000;
    // Return at once when bytes are there, else wait up to `ms` for the first one.
    COMMTIMEOUTS t{};
    t.ReadIntervalTimeout = MAXDWORD;
    t.ReadTotalTimeoutMultiplier = ms > 0 ? MAXDWORD : 0;
    t.ReadTotalTimeoutConstant = static_cast<DWORD>(ms);
    ::SetCommTimeouts(h, &t);
    std::memset(readOv_, 0, sizeof readOv_);
    ov(readOv_)->hEvent = static_cast<HANDLE>(readEvent_);
    DWORD got = 0;
    if (!::ReadFile(h, buf, static_cast<DWORD>(capacity), &got, ov(readOv_))) {
        if (::GetLastError() != ERROR_IO_PENDING) {
            osError_ = static_cast<int>(::GetLastError());
            return -1;
        }
        // Wait for data, the timeout or a wake. A wake cancels the read; bytes that arrived at the same
        // moment are still returned by GetOverlappedResult() below.
        HANDLE waits[2] = { static_cast<HANDLE>(readEvent_), nullptr };
        DWORD count = 1;
        if (waker_ != nullptr && waker_->valid()) {
            waits[1] = static_cast<HANDLE>(waker_->handle());
            count = 2;
        }
        const DWORD which = ::WaitForMultipleObjects(count, waits, FALSE, static_cast<DWORD>(ms) + 100);
        if (which != WAIT_OBJECT_0) {
            if (which == WAIT_OBJECT_0 + 1) {
                waker_->drain();
            }
            ::CancelIo(h);
        }
        if (!::GetOverlappedResult(h, ov(readOv_), &got, TRUE)) {
            if (::GetLastError() == ERROR_OPERATION_ABORTED) {
                return 0;
            }
            osError_ = static_cast<int>(::GetLastError());
            return -1;
        }
    }
    return static_cast<long>(got);
}

} // namespace canopen_conductor::slcan
