// Link test. A pseudo terminal plays the slcan adapter, so the whole path
// (open, frames in and out, wake, loss, reopen) runs on a plain Linux box.
#include "canopen_conductor/link.hpp"

#include <cerrno>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <poll.h>
#include <pty.h>
#include <thread>
#include <unistd.h>

using namespace canopen_conductor;
using namespace std::chrono_literals;

static int failures = 0;

#define CHECK(cond)                                                                \
    do {                                                                           \
        if (!(cond)) {                                                             \
            std::printf("FAIL line %d: %s\n", __LINE__, #cond);                    \
            ++failures;                                                            \
        }                                                                          \
    } while (0)

namespace {

struct Pty {
    int master = -1;
    char name[128] = {};
};

bool makePty(Pty& p) {
    int slave = -1;
    if (::openpty(&p.master, &slave, p.name, nullptr, nullptr) != 0) {
        return false;
    }
    ::close(slave);  // Link opens it by name; keep the master only
    // The slave name stays valid while the master is open.
    ::fcntl(p.master, F_SETFL, ::fcntl(p.master, F_GETFL) | O_NONBLOCK);
    return true;
}

// What the "adapter" has received so far, waiting up to `ms` for at least `want` bytes.
std::size_t readMaster(int fd, char* buf, std::size_t cap, std::size_t want, int ms) {
    std::size_t got = 0;
    const auto end = std::chrono::steady_clock::now() + std::chrono::milliseconds(ms);
    while (got < want && std::chrono::steady_clock::now() < end) {
        pollfd p{ fd, POLLIN, 0 };
        if (::poll(&p, 1, 20) > 0) {
            const ssize_t n = ::read(fd, buf + got, cap - got);
            if (n > 0) {
                got += static_cast<std::size_t>(n);
            }
        }
    }
    return got;
}

long elapsedMs(std::chrono::steady_clock::time_point since) {
    return static_cast<long>(
        std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - since).count());
}

} // namespace

int main() {
    CanFrame frame;

    // --- configuration and open errors ---
    {
        Link link;
        CHECK(link.state() == LinkState::Closed);
        CHECK(!link.open({ "", 500000 }));
        CHECK(link.lastError().code == LinkErrc::BadConfig);
        CHECK(!link.open({ "can0", 123456 }));
        CHECK(link.lastError().code == LinkErrc::BadConfig);

        CHECK(!link.open({ "/dev/doesnotexist", 500000 }));
        CHECK(link.lastError().code == LinkErrc::OpenFailed);
        CHECK(link.lastError().osError == ENOENT);
        CHECK(link.state() == LinkState::Closed);

        CHECK(!link.open({ "nosuchcan0", 500000 }));  // SocketCAN: no such interface
        CHECK(link.lastError().code == LinkErrc::OpenFailed);
        CHECK(link.lastError().osError != 0);

        char text[200];
        const auto n = Link::describe(link.lastError(), text, sizeof text);
        CHECK(n > 0 && std::strstr(text, "cannot open") != nullptr);
        std::printf("describe: %s\n", text);

        CHECK(!link.send(frame));
        CHECK(link.poll(frame, 1ms) == LinkEvent::Idle);  // not open: no wait, no error
        CHECK(link.close());                              // closing a closed link is fine
    }

    // --- a working link ---
    Pty pty;
    if (!makePty(pty)) {
        std::printf("cannot create a pty here, skipping the rest\n");
        return failures == 0 ? 0 : 1;
    }
    {
        Link link;
        CHECK(link.open({ pty.name, 500000 }));
        CHECK(link.state() == LinkState::Open);
        CHECK(link.lastError().code == LinkErrc::None);

        char buf[256] = {};
        const char expected[] = "C\rS6\rO\r";
        const auto got = readMaster(pty.master, buf, sizeof buf, sizeof expected - 1, 1000);
        CHECK(got == sizeof expected - 1 && std::memcmp(buf, expected, got) == 0);

        // a frame from the bus
        CHECK(::write(pty.master, "t1232AABB\r", 10) == 10);
        CHECK(link.poll(frame, 1000ms) == LinkEvent::Frame);
        CHECK(frame.id == 0x123 && frame.len == 2 && frame.data[0] == 0xAA && frame.data[1] == 0xBB);

        // a frame to the bus
        CanFrame out;
        out.id = 0x601;
        out.len = 1;
        out.data[0] = 0x40;
        CHECK(link.send(out));
        std::memset(buf, 0, sizeof buf);
        const auto sent = readMaster(pty.master, buf, sizeof buf, 8, 1000);
        CHECK(sent == 8 && std::memcmp(buf, "t601140\r", 8) == 0);

        // timeout: Idle, and it did wait
        auto t0 = std::chrono::steady_clock::now();
        CHECK(link.poll(frame, 60ms) == LinkEvent::Idle);
        const auto waited = elapsedMs(t0);
        CHECK(waited >= 50 && waited < 500);

        // wake from another thread while poll() waits
        t0 = std::chrono::steady_clock::now();
        std::thread waker([&] {
            std::this_thread::sleep_for(60ms);
            link.wake();
        });
        CHECK(link.poll(frame, 5000ms) == LinkEvent::Idle);
        CHECK(elapsedMs(t0) < 1000);
        waker.join();

        // a wake before poll() is not lost, and is used up once
        link.wake();
        t0 = std::chrono::steady_clock::now();
        CHECK(link.poll(frame, 5000ms) == LinkEvent::Idle);
        CHECK(elapsedMs(t0) < 200);
        t0 = std::chrono::steady_clock::now();
        CHECK(link.poll(frame, 60ms) == LinkEvent::Idle);
        CHECK(elapsedMs(t0) >= 50);

        // a frame and a wake together: the frame is not lost
        CHECK(::write(pty.master, "t1000\r", 6) == 6);
        link.wake();
        LinkEvent ev = link.poll(frame, 1000ms);
        if (ev == LinkEvent::Idle) {
            ev = link.poll(frame, 1000ms);
        }
        CHECK(ev == LinkEvent::Frame && frame.id == 0x100);

        // idle() is for a closed link: it waits for a wake, with a timeout
        {
            Link closed;
            auto i0 = std::chrono::steady_clock::now();
            CHECK(!closed.idle(60ms));
            CHECK(elapsedMs(i0) >= 50 && elapsedMs(i0) < 500);
            closed.wake();
            i0 = std::chrono::steady_clock::now();
            CHECK(closed.idle(5000ms));  // a wake before idle() is not lost
            CHECK(elapsedMs(i0) < 200);
            CHECK(!closed.idle(30ms));   // and it is used up
            std::thread w([&] {
                std::this_thread::sleep_for(60ms);
                closed.wake();
            });
            i0 = std::chrono::steady_clock::now();
            CHECK(closed.idle(5000ms));
            CHECK(elapsedMs(i0) < 1000);
            w.join();
        }

        // loss: the adapter goes away
        ::close(pty.master);
        pty.master = -1;
        CHECK(link.poll(frame, 1000ms) == LinkEvent::Lost);
        CHECK(link.state() == LinkState::Lost);
        CHECK(link.lastError().code == LinkErrc::Lost);
        CHECK(link.lastError().osError == EIO);
        char text[200];
        Link::describe(link.lastError(), text, sizeof text);
        std::printf("describe: %s\n", text);

        // reported once
        const auto t1 = std::chrono::steady_clock::now();
        CHECK(link.poll(frame, 1000ms) == LinkEvent::Idle);
        CHECK(elapsedMs(t1) < 100);
        CHECK(!link.send(out));

        // reopen on a new adapter
        Pty again;
        CHECK(makePty(again));
        CHECK(link.open({ again.name, 250000 }));
        CHECK(link.state() == LinkState::Open);
        CHECK(link.lastError().code == LinkErrc::None);
        std::memset(buf, 0, sizeof buf);
        const char expected2[] = "C\rS5\rO\r";
        const auto got2 = readMaster(again.master, buf, sizeof buf, sizeof expected2 - 1, 1000);
        CHECK(got2 == sizeof expected2 - 1 && std::memcmp(buf, expected2, got2) == 0);

        // close: ok, ends Closed, tells the adapter
        CHECK(link.close());
        CHECK(link.state() == LinkState::Closed);
        std::memset(buf, 0, sizeof buf);
        CHECK(readMaster(again.master, buf, sizeof buf, 2, 1000) == 2 && std::memcmp(buf, "C\r", 2) == 0);
        ::close(again.master);
    }

    std::printf(failures == 0 ? "link_test: all passed\n" : "link_test: %d FAILED\n", failures);
    return failures == 0 ? 0 : 1;
}
