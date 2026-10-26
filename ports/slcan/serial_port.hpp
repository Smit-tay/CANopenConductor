#pragma once

// Minimal serial line for the slcan port: POSIX (serial_posix.cpp) or
// Windows (serial_win32.cpp). Raw 8N1, no flow control.

#include "canopen_conductor/time.hpp"

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
    void close() noexcept;
    bool isOpen() const noexcept;

    // Without waiting: bytes taken by the OS (0 if none fit), -1 on error.
    long write(const char* data, std::size_t size) noexcept;
    // Waits up to `timeout` for data; bytes read, 0 on timeout, -1 on error.
    long read(char* buf, std::size_t capacity, Duration timeout) noexcept;

private:
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
