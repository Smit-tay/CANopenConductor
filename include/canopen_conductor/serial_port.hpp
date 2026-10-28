#pragma once

// Minimal serial line for the slcan port: POSIX (serial_posix.cpp) or
// Windows (serial_win32.cpp). Raw 8N1, no flow control.

#include "canopen_conductor/time.hpp"
#include "canopen_conductor/waker.hpp"

#include <cstddef>
#include <cstdint>

namespace canopen_conductor::slcan {

class SerialPort {
public:
    SerialPort() = default;
    ~SerialPort();
    SerialPort(const SerialPort&) = delete;
    SerialPort& operator=(const SerialPort&) = delete;

    // "/dev/ttyACM0" or "COM3". `baud` matters for real UARTs only.
    bool open(const char* device, unsigned baud) noexcept;
    // False if the OS reported an error while closing. The port is closed in any case.
    bool close() noexcept;
    bool isOpen() const noexcept;

    // The waker (not owned) that makes read() return early. Set it before open().
    void setWaker(Waker* waker) noexcept { waker_ = waker; }

    // The OS error (errno, GetLastError()) behind the last failure, 0 if none.
    int osError() const noexcept { return osError_; }

    // Without waiting: bytes taken by the OS (0 if none fit), -1 on error.
    long write(const char* data, std::size_t size) noexcept;
    // Waits up to `timeout` for data; bytes read, 0 on timeout or wake, -1 on error.
    long read(char* buf, std::size_t capacity, Duration timeout) noexcept;

private:
    Waker* waker_ = nullptr;
    int osError_ = 0;
#if defined(_WIN32)
    void* handle_ = nullptr;   // HANDLE
    void* readEvent_ = nullptr;
    void* writeEvent_ = nullptr;
    unsigned char readOv_[64] = {};   // OVERLAPPED storage
    unsigned char writeOv_[64] = {};
    bool writing_ = false;
    char writeBuf_[512] = {};
#else
    int fd_ = -1;
#endif
};

} // namespace canopen_conductor::slcan
