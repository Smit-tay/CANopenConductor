#pragma once

// Session: the life of one CAN connection, as the host sees it. Host layer, on top of Link.
//
// A Session answers three commands and tells the host what happened:
//
//  - connect():    open the interface, and keep trying until it opens (or until disconnect()).
//                  The first failed attempt is reported once (ConnectFailed). Later attempts stay quiet.
//                  A success is reported (Connected).
//  - reconnect():  the same, but only if a connection is wanted and nothing is open or being tried. This is
//                  what the host calls to recover after Lost. It does nothing after disconnect().
//  - disconnect(): close the interface, and stop trying. Reported (Disconnected) if it was open.
//
// While the interface is open, step() reads from it continuously. If it fails, that is reported once (Lost).
// A Session never reconnects by itself: it waits for the next connect() or reconnect().
//
// Like Link, a Session starts no thread and keeps no clock. The host runs step() in a loop on its own I/O thread.
// step() blocks (in the read while open, in a wait otherwise) and returns early, at once, when a command is
// given or wake() is called. The commands and wake() are for any thread. Everything else is for the I/O thread.
//
//     while (running) {
//         switch (session.step(1s)) { ... one case for each event ... }
//     }
//     session.close();
//
// The commands are one slot deep: if two are given before step() looks, the last one wins, except that reconnect()
// never replaces another command. That is enough because the host gives them one at a time.

#include "canopen_conductor/link.hpp"

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>

namespace canopen_conductor {

enum class SessionEvent : std::uint8_t {
    None,           // nothing to tell
    Connected,      // the interface is open
    ConnectFailed,  // the first failed attempt of a connect command. error() says why.
    Lost,           // the interface failed while open. error() says why.
    Disconnected,   // closed on request. error() is None, or the error the OS reported while closing.
};

class Session {
public:
    // The longest interface name configure() accepts.
    static constexpr std::size_t kMaxInterfaceName = 63;

    // The pause between two attempts of a connect command that has not succeeded yet.
    static constexpr Duration kRetryInterval = std::chrono::milliseconds(200);

    Session() noexcept = default;
    Session(const Session&) = delete;
    Session& operator=(const Session&) = delete;

    // The settings (the interface name is copied). False if they cannot be used: an empty or too long name, or a
    // bit rate that is not valid. I/O thread, or before step() is first called. Takes effect at the next attempt.
    bool configure(const LinkConfig& config) noexcept;

    // Commands. Any thread, any time, never block.
    void connect() noexcept;
    void reconnect() noexcept;
    void disconnect() noexcept;

    // Makes the step() that is waiting, or the next one, return early with None. This is how the host gets its
    // I/O thread to look at its own flags (a stop request, say). Any thread, any time.
    void wake() noexcept { link_.wake(); }

    // One turn of the loop. Waits at most `maxWait`. Returns at most one event.
    SessionEvent step(Duration maxWait) noexcept;

    // Closes the interface quietly (no event) and stops any connect command. I/O thread. False if the OS reported
    // an error while closing; the interface is closed anyway.
    bool close() noexcept;

    // The error of the event that step() returned last (None after Connected).
    LinkError error() const noexcept { return error_; }

    // Whether the connect command that is running, or ran last, came from reconnect() and not connect().
    bool viaReconnect() const noexcept { return viaReconnect_; }

    // The state of the interface, and whether a connect command is being carried out. I/O thread.
    LinkState linkState() const noexcept { return link_.state(); }
    bool connecting() const noexcept { return connecting_; }

    // How many frames have been received (and, for now, dropped) since the Session was made. Any thread.
    std::uint64_t rxFrames() const noexcept { return rxFrames_.load(std::memory_order_relaxed); }

private:
    enum class Request : std::uint8_t { None, Connect, Reconnect, Disconnect };

    void post(Request request) noexcept;
    void beginConnect(bool viaReconnect) noexcept;

    Link link_;
    LinkConfig config_;
    std::array<char, kMaxInterfaceName + 1> name_ {};

    std::atomic<Request> request_ { Request::None };
    std::atomic<std::uint64_t> rxFrames_ { 0 };

    // I/O thread only
    bool wanted_ = false;          // a connect command has been given, and no disconnect since
    bool connecting_ = false;      // a connect command is being carried out
    bool attempted_ = false;       // this command has made its first attempt
    bool failureReported_ = false; // this command has reported its first failure
    bool viaReconnect_ = false;
    LinkError error_;
};

} // namespace canopen_conductor
