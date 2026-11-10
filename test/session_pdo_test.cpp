// A raw PDO and the deferred SYNC producer, on a Session. A pseudo terminal plays the slcan adapter.
// Checks: a raw PDO frame is handed over whole through RawPdoFn, its deadline is still monitored (PdoFn with timeout),
// a raw config is refused with entries, and the SYNC producer sends nothing until startSync().
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

std::vector<RawPdoEvent> raws;
std::vector<PdoEvent> pdos;

void onRaw(void*, const RawPdoEvent& e) noexcept { raws.push_back(e); }
void onPdo(void*, const PdoEvent& e) noexcept { pdos.push_back(e); }

void stepFor(Session& s, int ms) {
    const auto t0 = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - t0 < std::chrono::milliseconds(ms)) {
        s.step(20ms);
    }
}

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
    CHECK(s.connect());
    drain(master);

    MasterConfig cfg;
    cfg.maxNodes = 4;
    cfg.maxPdos = 4;
    cfg.sync.period = 50ms;
    cfg.syncStarted = false;
    cfg.onRawPdo = onRaw;
    cfg.onPdo = onPdo;
    CHECK(s.startMaster(cfg));
    Master* m = s.master();
    CHECK(m != nullptr);
    if (m == nullptr) {
        return 1;
    }
    CHECK(!m->syncRunning());

    NodeConfig node;
    node.nodeId = 32;
    CHECK(m->addNode(node));

    // a raw PDO has no entries
    const PdoEntry entry { 0x2100, 0, DataType::Unsigned16 };
    PdoConfig bad;
    bad.nodeId = 32;
    bad.cobId = 0x1A0;
    bad.raw = true;
    bad.entries = &entry;
    bad.entryCount = 1;
    CHECK(!m->addPdo(bad));

    PdoConfig pdo;
    pdo.nodeId = 32;
    pdo.cobId = 0x1A0;
    pdo.raw = true;
    pdo.timeout = 200ms;
    CHECK(m->addPdo(pdo));
    CHECK(!m->addPdo(pdo));  // the COB-ID is taken now

    // no SYNC while it is not started
    stepFor(s, 300);
    CHECK(count(drain(master), "t0800") == 0);

    // started: SYNCs leave on 080h, 50 ms apart
    m->startSync();
    CHECK(m->syncRunning());
    stepFor(s, 400);
    const auto syncs = count(drain(master), "t0800");
    std::printf("syncs in 400 ms: %zu\n", syncs);
    CHECK(syncs >= 6 && syncs <= 9);

    // a raw frame comes out whole
    CHECK(::write(master, "t1A021122\r", 10) == 10);
    stepFor(s, 50);
    CHECK(raws.size() == 1);
    if (!raws.empty()) {
        CHECK(raws[0].nodeId == 32);
        CHECK(raws[0].cobId == 0x1A0);
        CHECK(raws[0].len == 2);
        CHECK(raws[0].data[0] == 0x11 && raws[0].data[1] == 0x22);
    }
    CHECK(pdos.empty());

    // then silence: the deadline is reported once, as a timeout
    stepFor(s, 400);
    CHECK(pdos.size() == 1);
    if (!pdos.empty()) {
        CHECK(pdos[0].timeout);
        CHECK(pdos[0].cobId == 0x1A0);
    }

    // stopped again: no more SYNC
    m->stopSync();
    drain(master);
    stepFor(s, 200);
    CHECK(count(drain(master), "t0800") == 0);

    CHECK(s.close());
    ::close(master);
    std::printf(failures == 0 ? "session_pdo_test: OK\n" : "session_pdo_test: %d FAILED\n", failures);
    return failures == 0 ? 0 : 1;
}
