// Session test. A pseudo terminal plays the slcan adapter, and a symlink that appears and disappears plays the
// adapter being plugged in and out, so the whole life of a connection runs on a plain Linux box.
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

// Runs step() until it returns `wanted` (true) or `limit` has passed (false). Counts every other event it sees
// in `others`, so a test can say "nothing else happened".
bool stepUntil(Session& s, SessionEvent wanted, int limitMs, int* others = nullptr) {
    const auto t0 = std::chrono::steady_clock::now();
    while (elapsedMs(t0) < limitMs) {
        const SessionEvent e = s.step(100ms);
        if (e == wanted) {
            return true;
        }
        if (e != SessionEvent::None && others != nullptr) {
            ++*others;
        }
    }
    return false;
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
        // reconnect() before any connect() does nothing: no connection is wanted yet
        idle.reconnect();
        CHECK(stepFor(idle, 300) == 0);
        CHECK(!idle.connecting());
    }

    Pty pty;
    if (!makePty(pty)) {
        std::printf("cannot create a pty here, skipping the rest\n");
        return failures == 0 ? 0 : 1;
    }
    Plug plug;
    CHECK(s.configure({ plug.path.c_str(), 500000 }));

    // --- connect while the adapter is unplugged: fails once, keeps trying quietly ---
    s.connect();
    CHECK(s.step(100ms) == SessionEvent::ConnectFailed);
    CHECK(s.error().code == LinkErrc::OpenFailed);
    CHECK(s.error().osError == ENOENT);
    CHECK(!s.viaReconnect());
    CHECK(s.connecting());
    CHECK(stepFor(s, 700) == 0);  // several attempts, no further report
    CHECK(s.connecting());
    CHECK(s.linkState() == LinkState::Closed);

    // --- plug it in: the very command that was failing now succeeds ---
    plug.plugIn(pty.name);
    CHECK(stepUntil(s, SessionEvent::Connected, 2000));
    CHECK(s.error().code == LinkErrc::None);
    CHECK(!s.connecting());
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
    CHECK(stepUntil(s, SessionEvent::Lost, 2000));
    CHECK(s.error().code == LinkErrc::Lost);
    CHECK(s.error().osError == EIO);
    CHECK(s.linkState() == LinkState::Lost);
    CHECK(stepFor(s, 800) == 0);
    CHECK(s.linkState() == LinkState::Lost);
    CHECK(!s.connecting());

    // --- reconnect(): while it is still unplugged it fails once (and says it came from reconnect()) ---
    s.reconnect();
    CHECK(s.step(100ms) == SessionEvent::ConnectFailed);
    CHECK(s.viaReconnect());
    CHECK(s.error().osError == ENOENT);
    CHECK(stepFor(s, 500) == 0);

    // --- a second reconnect() while one is running does nothing ---
    s.reconnect();
    CHECK(stepFor(s, 300) == 0);

    // --- plug in a new adapter: the same command connects ---
    Pty again;
    CHECK(makePty(again));
    plug.plugIn(again.name);
    CHECK(stepUntil(s, SessionEvent::Connected, 2000));
    CHECK(s.linkState() == LinkState::Open);

    // --- reconnect() or connect() while open: nothing happens ---
    s.reconnect();
    s.connect();
    CHECK(stepFor(s, 300) == 0);
    CHECK(s.linkState() == LinkState::Open);

    // --- disconnect(): closed, reported, and reconnect() does nothing afterwards ---
    s.disconnect();
    CHECK(s.step(500ms) == SessionEvent::Disconnected);
    CHECK(s.error().code == LinkErrc::None);
    CHECK(s.linkState() == LinkState::Closed);
    s.reconnect();
    CHECK(stepFor(s, 400) == 0);
    CHECK(s.linkState() == LinkState::Closed);
    CHECK(!s.connecting());

    // --- connect() starts it again, as the first time ---
    CHECK(::write(again.master, "", 0) == 0);
    s.connect();
    CHECK(stepUntil(s, SessionEvent::Connected, 2000));
    CHECK(s.linkState() == LinkState::Open);

    // --- a disconnect while a connect command is still trying ends it, with no report ---
    s.disconnect();
    CHECK(s.step(500ms) == SessionEvent::Disconnected);
    plug.unplug();
    s.connect();
    CHECK(s.step(100ms) == SessionEvent::ConnectFailed);
    CHECK(s.connecting());
    s.disconnect();
    CHECK(s.step(100ms) == SessionEvent::None);
    CHECK(!s.connecting());
    plug.plugIn(again.name);
    CHECK(stepFor(s, 600) == 0);  // it is plugged in, but nobody is trying
    CHECK(s.linkState() == LinkState::Closed);

    // --- connect() from another thread gets step() out of its wait at once ---
    {
        const auto t0 = std::chrono::steady_clock::now();
        std::thread t([&] {
            std::this_thread::sleep_for(80ms);
            s.connect();
        });
        SessionEvent e = SessionEvent::None;
        while (e != SessionEvent::Connected && elapsedMs(t0) < 3000) {
            e = s.step(5000ms);
        }
        CHECK(e == SessionEvent::Connected);
        CHECK(elapsedMs(t0) < 1500);
        t.join();
    }

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

    // --- close(): quiet, and ends it ---
    CHECK(s.close());
    CHECK(s.linkState() == LinkState::Closed);
    CHECK(stepFor(s, 300) == 0);

    ::close(again.master);
    std::printf(failures == 0 ? "session_test: all passed\n" : "session_test: %d FAILED\n", failures);
    return failures == 0 ? 0 : 1;
}
