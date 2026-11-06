#pragma once

// Session: one CAN connection, as the host drives it. Host layer, on top of Link.
//
// A Session is mechanism only. It makes no decision: it never retries, never reconnects, and never remembers that a
// connection was wanted. The host owns the state machine (when to connect, when to give up, what to do after a
// loss) and calls these functions in the order it decides. The Session only refuses calls that make no sense in
// its own state (the link states Closed, Open and Lost), and says why.
//
//  - connect():  one attempt to open the interface. Returns the result. Refused while the link is open.
//  - close():    closes the interface. Always ends Closed. Valid in every state.
//  - step():     while the interface is open, reads from it; if it fails, that is reported once (Lost), and the
//                link is Lost from then on. Otherwise it only waits. step() returns early when wake() is called.
//
// Like Link, a Session starts no thread and keeps no clock for the protocol. The host runs step() in a loop on its
// own I/O thread, and calls connect() and close() from that same thread. Only wake() may be called from another one.
//
//     while (running) {
//         switch (session.step(1s)) { case SessionEvent::Lost: ... }
//     }
//     session.close();
//
// A Session can also carry a Master (startMaster()). While one runs, step() does what the Master needs on the
// I/O thread: it hands every received frame to Master::onFrame(), calls Master::tick() after each turn, and
// shortens its wait so that the turn ends when the Master next has work (heartbeat due, timeout due). The Master's
// callbacks therefore run inside step(), on the I/O thread, and must only signal. The Master is stopped by
// close(), by connect() and by a loss: it never outlives the open link it was started on.
//
// One exception to "no clock": connect() reads std::chrono::steady_clock to apply its timeout. The operating system
// calls that open a port cannot be interrupted, so the timeout is applied after the fact: an open that took longer
// than allowed is closed again and reported as a failure (LinkErrc::Timeout). It bounds what the host accepts, not
// how long the OS call blocks.

#include "canopen_conductor/link.hpp"
#include "canopen_conductor/master.hpp"

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>

namespace canopen_conductor {

enum class SessionEvent : std::uint8_t {
    None,  // nothing to tell
    Lost,  // the interface failed while open. error() says why. Reported once.
};

class Session {
public:
    // The longest interface name configure() accepts.
    static constexpr std::size_t kMaxInterfaceName = 63;

    // The default for connect(): how long an open may take before it counts as failed.
    static constexpr Duration kDefaultConnectTimeout = std::chrono::seconds(2);

    Session() noexcept = default;
    ~Session() { stopMaster(); }
    Session(const Session&) = delete;
    Session& operator=(const Session&) = delete;

    // The settings (the interface name is copied). False if they cannot be used: an empty or too long name, or a
    // bit rate that is not valid. I/O thread, or before step() is first called. Takes effect at the next connect().
    bool configure(const LinkConfig& config) noexcept;

    // One attempt to open the interface. I/O thread. True if it is open. On false, error() says why:
    // WrongState (it is open already), BadConfig, NoResources, OpenFailed, or Timeout (the open worked, but took
    // longer than `timeout`, so the interface has been closed again). The link is Closed after a failure.
    bool connect(Duration timeout = kDefaultConnectTimeout) noexcept;

    // Closes the interface. I/O thread. Valid in every state, and the link is Closed afterwards. False if the OS
    // reported an error while closing (error().code == CloseFailed); the interface is closed anyway.
    bool close() noexcept;

    // Makes the step() that is waiting, or the next one, return early with None. This is how the host gets its
    // I/O thread to look at its own flags (a command, a stop request). Any thread, any time.
    void wake() noexcept { link_.wake(); }

    // One turn of the loop. I/O thread. Waits at most `maxWait`: in the read while the interface is open, in a
    // wait otherwise. Returns at most one event.
    SessionEvent step(Duration maxWait) noexcept;

    // The error of the call that failed last (connect(), close()), or of the Lost that step() returned last.
    // None after a call that succeeded.
    LinkError error() const noexcept { return error_; }

    // Builds a Master over the open link and runs it from step() on. I/O thread. `config` is copied by the
    // Master (its node and PDO tables), the callbacks and context in it stay yours. False if the link is not open,
    // a Master runs already, the config is invalid, or there is no memory. Memory is taken here, never in step().
    bool startMaster(const MasterConfig& config) noexcept;

    // Stops the Master, if one runs. I/O thread.
    void stopMaster() noexcept;

    // The running Master, or nullptr. I/O thread only. Valid until stopMaster(), close(), connect() or a loss.
    Master* master() noexcept { return master_; }

    // Closed, Open (working) or Lost (was open, the port failed). I/O thread.
    LinkState linkState() const noexcept { return link_.state(); }

    // How many frames have been received since the Session was made. Any thread.
    std::uint64_t rxFrames() const noexcept { return rxFrames_.load(std::memory_order_relaxed); }

private:
    static TimePoint now() noexcept;

    Link link_;
    Master* master_ = nullptr;
    void* masterMemory_ = nullptr;
    LinkConfig config_;
    std::array<char, kMaxInterfaceName + 1> name_ {};

    std::atomic<std::uint64_t> rxFrames_ { 0 };
    LinkError error_;
};

} // namespace canopen_conductor
