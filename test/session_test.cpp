// Session test. A pseudo terminal plays the slcan adapter, and a symlink that appears and disappears plays the
// adapter being plugged in and out, so the whole life of a connection runs on a plain Linux box.
// The Session decides nothing: every attempt here is one connect() call, and nothing is ever retried.
#include "canopen_conductor/session.hpp"

#include <cerrno>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <pty.h>
#include <string>
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
    ::close(slave);
    ::fcntl(p.master, F_SETFL, ::fcntl(p.master, F_GETFL) | O_NONBLOCK);
    return true;
}

long elapsedMs(std::chrono::steady_clock::time_point since) {
    return static_cast<long>(
        std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - since).count());
}

// Runs step() for `ms` and returns how many events (other than None) it saw.
int stepFor(Session& s, int ms) {
    int events = 0;
    const auto t0 = std::chrono::steady_clock::now();
    while (elapsedMs(t0) < ms) {
        if (s.step(50ms) != SessionEvent::None) {
            ++events;
        }
    }
    return events;
}

// The "adapter plug": a path that exists only while an adapter is plugged in.
struct Plug {
    std::string path;
    Plug() : path("/tmp/session_test_" + std::to_string(::getpid()) + ".tty") { ::unlink(path.c_str()); }
    ~Plug() { ::unlink(path.c_str()); }
    void plugIn(const char* ptyName) {
        ::unlink(path.c_str());
        CHECK(::symlink(ptyName, path.c_str()) == 0);
    }
    void unplug() { ::unlink(path.c_str()); }
};

} // namespace

int main() {
    Session s;

    // --- configure ---
    CHECK(!s.configure({ "", 500000 }));
    CHECK(!s.configure({ "can0", 123456 }));
    CHECK(!s.configure({ std::string(Session::kMaxInterfaceName + 1, 'x').c_str(), 500000 }));
    CHECK(s.configure({ std::string(Session::kMaxInterfaceName, 'x').c_str(), 500000 }));

    // --- nothing was asked: step() just waits, and reports nothing ---
    {
        Session idle;
        CHECK(idle.configure({ "can0", 500000 }));
        const auto t0 = std::chrono::steady_clock::now();
        CHECK(idle.step(60ms) == SessionEvent::None);
        CHECK(elapsedMs(t0) >= 50 && elapsedMs(t0) < 500);
        CHECK(idle.linkState() == LinkState::Closed);
        // close() on a closed link is harmless
        CHECK(idle.close());
    }

    // --- connect() without a valid configure(): refused, state unchanged ---
    {
        Session unconfigured;
        CHECK(!unconfigured.connect());
        CHECK(unconfigured.error().code == LinkErrc::BadConfig);
        CHECK(unconfigured.linkState() == LinkState::Closed);
    }

    Pty pty;
    if (!makePty(pty)) {
        std::printf("cannot create a pty here, skipping the rest\n");
        return failures == 0 ? 0 : 1;
    }
    Plug plug;
    CHECK(s.configure({ plug.path.c_str(), 500000 }));

    // --- connect while the adapter is unplugged: ONE attempt, it fails, and nothing is tried again ---
    CHECK(!s.connect());
    CHECK(s.error().code == LinkErrc::OpenFailed);
    CHECK(s.error().osError == ENOENT);
    CHECK(s.linkState() == LinkState::Closed);
    CHECK(stepFor(s, 700) == 0);
    CHECK(s.linkState() == LinkState::Closed);  // still closed: it did not retry by itself

    // --- plug it in: the next connect() succeeds ---
    plug.plugIn(pty.name);
    CHECK(s.connect());
    CHECK(s.error().code == LinkErrc::None);
    CHECK(s.linkState() == LinkState::Open);

    // --- connect() while open is refused, and changes nothing ---
    CHECK(!s.connect());
    CHECK(s.error().code == LinkErrc::WrongState);
    CHECK(s.linkState() == LinkState::Open);

    // --- while open it reads continuously ---
    CHECK(::write(pty.master, "t1232AABB\r", 10) == 10);
    CHECK(::write(pty.master, "t1000\r", 6) == 6);
    CHECK(stepFor(s, 300) == 0);
    CHECK(s.rxFrames() == 2);

    // --- the adapter goes away: Lost, once, and no reconnect on its own ---
    ::close(pty.master);
    pty.master = -1;
    plug.unplug();
    bool lost = false;
    for (int i = 0; i < 40 && !lost; ++i) {
        lost = s.step(50ms) == SessionEvent::Lost;
    }
    CHECK(lost);
    CHECK(s.error().code == LinkErrc::Lost);
    CHECK(s.error().osError == EIO);
    CHECK(s.linkState() == LinkState::Lost);
    CHECK(stepFor(s, 800) == 0);
    CHECK(s.linkState() == LinkState::Lost);

    // --- connect() from Lost: while still unplugged it fails once; the state becomes Closed ---
    CHECK(!s.connect());
    CHECK(s.error().osError == ENOENT);
    CHECK(s.linkState() == LinkState::Closed);

    // --- plug in a new adapter: connect() works from the failed state ---
    Pty again;
    CHECK(makePty(again));
    plug.plugIn(again.name);
    CHECK(s.connect());
    CHECK(s.linkState() == LinkState::Open);

    // --- close(): closed, no error, valid again afterwards ---
    CHECK(s.close());
    CHECK(s.error().code == LinkErrc::None);
    CHECK(s.linkState() == LinkState::Closed);
    CHECK(s.close());

    // --- close() from Lost ends in Closed too ---
    CHECK(s.connect());
    ::close(again.master);
    again.master = -1;
    plug.unplug();
    lost = false;
    for (int i = 0; i < 40 && !lost; ++i) {
        lost = s.step(50ms) == SessionEvent::Lost;
    }
    CHECK(lost);
    CHECK(s.close());
    CHECK(s.linkState() == LinkState::Closed);

    // --- the timeout: an open that takes longer than allowed is closed again and reported as Timeout.
    //     A zero timeout makes any open "too slow", which is the only way to test it without a slow device. ---
    Pty third;
    CHECK(makePty(third));
    plug.plugIn(third.name);
    CHECK(!s.connect(Duration::zero()));
    CHECK(s.error().code == LinkErrc::Timeout);
    CHECK(s.linkState() == LinkState::Closed);
    // and with the default timeout the same open works
    CHECK(s.connect());
    CHECK(s.linkState() == LinkState::Open);

    // --- wake() gets step() out of the blocking read ---
    {
        const auto t0 = std::chrono::steady_clock::now();
        std::thread t([&] {
            std::this_thread::sleep_for(80ms);
            s.wake();
        });
        CHECK(s.step(5000ms) == SessionEvent::None);
        CHECK(elapsedMs(t0) < 1500);
        t.join();
    }

    // --- wake() gets step() out of the wait while closed, too ---
    CHECK(s.close());
    {
        const auto t0 = std::chrono::steady_clock::now();
        std::thread t([&] {
            std::this_thread::sleep_for(80ms);
            s.wake();
        });
        CHECK(s.step(5000ms) == SessionEvent::None);
        CHECK(elapsedMs(t0) < 1500);
        t.join();
    }

    ::close(third.master);
    std::printf(failures == 0 ? "session_test: all passed\n" : "session_test: %d FAILED\n", failures);
    return failures == 0 ? 0 : 1;
}
