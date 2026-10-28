#pragma once

#include "canopen_conductor/time.hpp"

// A wake-up for a thread that is blocked in a port's receive(): any thread
// calls wake(), and the receiving thread returns early to look at its work.
//
// A wake that arrives while nobody is waiting is not lost: the next wait
// returns at once, once. A loop that checks its flags after every return
// from receive() therefore never misses a request.

namespace canopen_conductor {

class Waker {
public:
    Waker() noexcept;
    ~Waker();
    Waker(const Waker&) = delete;
    Waker& operator=(const Waker&) = delete;

    // False if the OS refused to create it (out of descriptors or handles).
    bool valid() const noexcept;

    // Any thread, at any time. Never blocks.
    void wake() noexcept;

    // The waiting thread, after it has seen a wake: makes the next wait block again.
    void drain() noexcept;

    // Waits up to `timeout` for a wake, and consumes it. True if there was one. This is for a thread that has
    // nothing else to wait for.
    bool wait(Duration timeout) noexcept;

#if defined(_WIN32)
    void* handle() const noexcept { return event_; }  // a manual-reset event, signalled while a wake is pending
#else
    int fd() const noexcept { return fds_[0]; }       // readable while a wake is pending
#endif

private:
#if defined(_WIN32)
    void* event_ = nullptr;
#else
    int fds_[2] = { -1, -1 };
#endif
};

} // namespace canopen_conductor
