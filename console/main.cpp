// canopen-console: try the CAN connection by hand, on real hardware.
//
//     canopen-console <interface> [bitrate]
//
// <interface> is "COM3" or "/dev/ttyACM0" for an slcan adapter, or "can0" for a SocketCAN interface (Linux).
// The bitrate is in bit/s and defaults to 500000.
//
// It runs the Session the same way a host does: one I/O thread calls step() in a loop. The Session decides nothing,
// so this console is the state machine: a typed command is a request, the I/O thread carries it out. CONNECT makes
// one attempt and reports it. Nothing is ever retried. Type HELP.
#include "canopen_conductor/session.hpp"

#include "terminal.hpp"

#include <algorithm>
#include <atomic>
#include <cctype>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <mutex>
#include <sstream>
#include <stop_token>
#include <string>
#include <thread>
#include <vector>

using namespace canopen_conductor;
using namespace std::chrono_literals;

namespace {

// What the commands and the I/O thread share
struct App {
    console::Terminal* terminal = nullptr;
    Session session;
    std::string interface;
    unsigned bitrate = 500000;

    // Written by the I/O thread, read by the command thread
    std::atomic<const char*> state { "closed" };
    // Requests from the command thread. DISCONNECT is checked first, so it always wins.
    std::atomic<bool> connectRequested { false };
    std::atomic<bool> disconnectRequested { false };
    std::atomic<unsigned> connectedCount { 0 };
    std::atomic<unsigned> failedCount { 0 };
    std::atomic<unsigned> lostCount { 0 };
    std::mutex errorMutex;
    std::string lastError;

    void print(const std::string& line) { terminal->printLine(line); }
};

std::string describe(const LinkError& error) {
    char text[256];
    Link::describe(error, text, sizeof text);
    return text;
}

std::string toUpper(std::string s) {
    std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) { return static_cast<char>(std::toupper(c)); });
    return s;
}

// --- the I/O thread ---

void setState(App& app) {
    app.state = app.session.linkState() == LinkState::Open   ? "open"
              : app.session.linkState() == LinkState::Lost ? "lost"
                                                           : "closed";
}

void ioThread(std::stop_token stop, App& app) {
    // A stop request must also end a step() that is waiting
    std::stop_callback onStop(stop, [&app] { app.session.wake(); });

    while (!stop.stop_requested()) {
        // DISCONNECT first: it is always obeyed, and it drops a CONNECT that was given before it was looked at
        if (app.disconnectRequested.exchange(false)) {
            app.connectRequested = false;
            const bool wasOpen = app.session.linkState() == LinkState::Open;
            const bool closed = app.session.close();
            if (wasOpen) {
                app.print(closed ? std::string("DISCONNECTED")
                                 : "DISCONNECTED: " + describe(app.session.error()));
            } else {
                app.print("DISCONNECTED (nothing was open)");
            }
            std::lock_guard<std::mutex> lock(app.errorMutex);
            app.lastError.clear();
        }

        if (app.connectRequested.exchange(false)) {
            if (app.session.connect()) {
                ++app.connectedCount;
                {
                    std::lock_guard<std::mutex> lock(app.errorMutex);
                    app.lastError.clear();
                }
                app.print("CONNECTED " + app.interface);
            } else {
                const std::string text = describe(app.session.error());
                if (app.session.error().code == LinkErrc::WrongState) {
                    app.print("REJECTED: already connected. DISCONNECT first.");
                } else {
                    ++app.failedCount;
                    app.print("CONNECT FAILED: " + text + " (nothing is tried again; type CONNECT)");
                    std::lock_guard<std::mutex> lock(app.errorMutex);
                    app.lastError = text;
                }
            }
        }

        if (app.session.step(1s) == SessionEvent::Lost) {
            ++app.lostCount;
            const std::string text = describe(app.session.error());
            app.print("LOST: " + text + " (nothing is tried again; type CONNECT)");
            std::lock_guard<std::mutex> lock(app.errorMutex);
            app.lastError = text;
        }
        setState(app);
    }
    app.session.close();
}

// --- the commands ---

enum class Result { Ok, Quit };

struct Command {
    const char* verb;
    const char* usage;
    const char* summary;
    Result (*run)(App&, const std::vector<std::string>& args);
};

extern const Command kCommands[];
extern const std::size_t kCommandCount;

Result cmdHelp(App& app, const std::vector<std::string>& args) {
    if (args.size() > 1) {
        const std::string wanted = toUpper(args[1]);
        for (std::size_t i = 0; i < kCommandCount; ++i) {
            if (wanted == kCommands[i].verb) {
                app.print(std::string("  ") + kCommands[i].usage);
                app.print(std::string("      ") + kCommands[i].summary);
                return Result::Ok;
            }
        }
        app.print("no command '" + args[1] + "'. Type HELP for the list.");
        return Result::Ok;
    }
    app.print("Commands (HELP <command> for details; several can be given on one line, separated by ';'):");
    for (std::size_t i = 0; i < kCommandCount; ++i) {
        std::string line = std::string("  ") + kCommands[i].verb;
        line.resize(std::max<std::size_t>(line.size(), 14), ' ');
        app.print(line + kCommands[i].summary);
    }
    return Result::Ok;
}

Result cmdStatus(App& app, const std::vector<std::string>&) {
    std::string error;
    {
        std::lock_guard<std::mutex> lock(app.errorMutex);
        error = app.lastError;
    }
    app.print(std::string("state=") + app.state.load() + " interface=" + app.interface + " bitrate=" +
              std::to_string(app.bitrate) + " rxFrames=" + std::to_string(app.session.rxFrames()) +
              " connected=" + std::to_string(app.connectedCount.load()) +
              " connectFailed=" + std::to_string(app.failedCount.load()) +
              " lost=" + std::to_string(app.lostCount.load()));
    if (!error.empty()) {
        app.print("lastError=" + error);
    }
    return Result::Ok;
}

Result cmdConnect(App& app, const std::vector<std::string>&) {
    app.connectRequested = true;
    app.session.wake();
    app.print("OK: connect requested (one attempt)");
    return Result::Ok;
}

Result cmdDisconnect(App& app, const std::vector<std::string>&) {
    app.disconnectRequested = true;
    app.session.wake();
    app.print("OK: disconnect requested");
    return Result::Ok;
}

Result cmdQuit(App&, const std::vector<std::string>&) {
    return Result::Quit;
}

// The one list of commands. HELP is made from it, so a new command is one more row.
const Command kCommands[] = {
    { "HELP", "HELP [command]", "show the commands, or the usage of one", cmdHelp },
    { "STATUS", "STATUS", "state of the connection, frame count, event counts, last error", cmdStatus },
    { "CONNECT", "CONNECT", "one attempt to open the interface (rejected if it is open)",
      cmdConnect },
    { "DISCONNECT", "DISCONNECT", "close the interface; always obeyed", cmdDisconnect },
    { "QUIT", "QUIT", "close the interface and leave", cmdQuit },
};
const std::size_t kCommandCount = sizeof kCommands / sizeof kCommands[0];

std::vector<std::string> tokenize(const std::string& line) {
    std::vector<std::string> tokens;
    std::istringstream stream(line);
    std::string token;
    while (stream >> token) {
        tokens.push_back(token);
    }
    return tokens;
}

Result runLine(App& app, const std::string& line) {
    std::istringstream commands(line);
    std::string one;
    while (std::getline(commands, one, ';')) {
        const std::vector<std::string> args = tokenize(one);
        if (args.empty()) {
            continue;
        }
        const std::string verb = toUpper(args[0]);
        bool found = false;
        for (std::size_t i = 0; i < kCommandCount; ++i) {
            if (verb == kCommands[i].verb) {
                found = true;
                if (kCommands[i].run(app, args) == Result::Quit) {
                    return Result::Quit;
                }
                break;
            }
        }
        if (!found) {
            app.print("ERR: unknown command '" + args[0] + "'. Type HELP.");
        }
    }
    return Result::Ok;
}

} // namespace

int main(int argc, char* argv[]) {
    if (argc < 2 || argc > 3 || std::string(argv[1]) == "--help" || std::string(argv[1]) == "-h") {
        std::printf("usage: canopen-console <interface> [bitrate]\n"
                    "  <interface>  COM3, /dev/ttyACM0 (slcan adapter) or can0 (SocketCAN, Linux)\n"
                    "  [bitrate]    bit/s, default 500000\n");
        return argc < 2 ? 1 : 0;
    }

    // The terminal is never destroyed on purpose: its input thread may still be waiting for a key when the
    // program ends, and must not find its object gone.
    auto* terminal = new console::Terminal;

    App app;
    app.terminal = terminal;
    app.interface = argv[1];
    if (argc == 3) {
        char* end = nullptr;
        const unsigned long value = std::strtoul(argv[2], &end, 10);
        if (end == argv[2] || *end != '\0' || value > 10000000UL) {
            std::fprintf(stderr, "error: the bitrate must be a number of bit/s, got '%s'\n", argv[2]);
            return 1;
        }
        app.bitrate = static_cast<unsigned>(value);
    }
    if (!app.session.configure({ app.interface.c_str(), app.bitrate })) {
        std::fprintf(stderr,
                     "error: unusable settings. The bitrate must be one of 10000, 20000, 50000, 100000, 125000, "
                     "250000, 500000, 800000, 1000000 and the interface name at most %zu characters.\n",
                     Session::kMaxInterfaceName);
        return 1;
    }

    terminal->start("canopen-console - type HELP for the commands");
    app.print("canopen-console: " + app.interface + " @ " + std::to_string(app.bitrate) +
              " bit/s. Nothing is open yet: type CONNECT. HELP lists the commands.");

    std::jthread io([&app](std::stop_token stop) { ioThread(stop, app); });

    std::string line;
    while (terminal->nextLine(line)) {
        if (runLine(app, line) == Result::Quit) {
            break;
        }
    }

    io.request_stop();
    io.join();
    terminal->stop();
    std::printf("canopen-console finished\n");
    return 0;
}
