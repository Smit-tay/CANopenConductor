// A Master on a Session. A pseudo terminal plays the slcan adapter.
// Checks: the master's heartbeat leaves on 701h at its period, a node heartbeat on 720h comes out as node events,
// its silence as HeartbeatLost, and the master is gone after close().
#include "canopen_conductor/session.hpp"

#include <chrono>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <pty.h>
#include <string>
#include <unistd.h>
#include <vector>

using namespace canopen_conductor;
using namespace std::chrono_literals;

static int failures = 0;
#define CHECK(cond)                                                        \
    do {                                                                   \
        if (!(cond)) {                                                     \
            std::printf("FAIL line %d: %s\n", __LINE__, #cond);            \
            ++failures;                                                    \
        }                                                                  \
    } while (0)

namespace {

std::vector<NodeEvent> events;

void onNode(void*, const NodeEvent& e) noexcept { events.push_back(e); }

void stepFor(Session& s, int ms) {
    const auto t0 = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - t0 < std::chrono::milliseconds(ms)) {
        s.step(50ms);
    }
}

// Reads whatever the "adapter" has received, without blocking.
std::string drain(int fd) {
    std::string out;
    char buf[256];
    for (;;) {
        const ssize_t n = ::read(fd, buf, sizeof buf);
        if (n <= 0) {
            break;
        }
        out.append(buf, static_cast<std::size_t>(n));
    }
    return out;
}

std::size_t count(const std::string& text, const char* needle) {
    std::size_t n = 0;
    for (auto p = text.find(needle); p != std::string::npos; p = text.find(needle, p + 1)) {
        ++n;
    }
    return n;
}

} // namespace

int main() {
    int master = -1, slave = -1;
    char name[128] = {};
    if (::openpty(&master, &slave, name, nullptr, nullptr) != 0) {
        std::printf("no pty here, skipping\n");
        return 0;
    }
    ::close(slave);
    ::fcntl(master, F_SETFL, ::fcntl(master, F_GETFL) | O_NONBLOCK);

    Session s;
    CHECK(s.configure({ name, 250000 }));

    NodeConfig node;
    node.nodeId = 32;
    node.heartbeatTimeout = 300ms;
    MasterConfig cfg;
    cfg.nodes = &node;
    cfg.nodeCount = 1;
    cfg.nmt.heartbeatNodeId = 1;
    cfg.nmt.heartbeatPeriod = 100ms;
    cfg.onNodeEvent = onNode;

    // not open: refused
    CHECK(!s.startMaster(cfg));
    CHECK(s.master() == nullptr);

    CHECK(s.connect());
    drain(master);  // the adapter's setup commands
    CHECK(s.startMaster(cfg));
    CHECK(s.master() != nullptr);
    CHECK(!s.startMaster(cfg));  // one at a time

    // the master's heartbeat: 1 s at 100 ms is about 10 frames, state 05
    stepFor(s, 1000);
    std::string tx = drain(master);
    const auto beats = count(tx, "t701105");
    std::printf("master heartbeats in 1 s: %zu\n", beats);
    CHECK(beats >= 8 && beats <= 11);

    // a node heartbeat: first one is a state change to Operational
    CHECK(::write(master, "t720105\r", 8) == 8);
    stepFor(s, 100);
    CHECK(!events.empty());
    if (!events.empty()) {
        CHECK(events.back().nodeId == 32);
        CHECK(events.back().state == NmtState::Operational);
    }
    CHECK(s.master()->node(32) != nullptr && s.master()->node(32)->state == NmtState::Operational);

    // silence: HeartbeatLost after the timeout
    events.clear();
    stepFor(s, 600);
    bool lost = false;
    for (const auto& e : events) {
        lost = lost || e.kind == NodeEventKind::HeartbeatLost;
    }
    CHECK(lost);

    // it comes back
    events.clear();
    CHECK(::write(master, "t720105\r", 8) == 8);
    stepFor(s, 100);
    bool resumed = false;
    for (const auto& e : events) {
        resumed = resumed || e.kind == NodeEventKind::HeartbeatResumed;
    }
    CHECK(resumed);

    // close() stops the master, and a new one can be started after connect()
    CHECK(s.close());
    CHECK(s.master() == nullptr);
    CHECK(s.connect());
    CHECK(s.startMaster(cfg));
    s.stopMaster();
    CHECK(s.master() == nullptr);

    ::close(master);
    std::printf(failures == 0 ? "session_master_test: OK\n" : "session_master_test: %d FAILED\n", failures);
    return failures == 0 ? 0 : 1;
}
