#pragma once

// Terminal: a split screen for a console program. The bottom three rows are fixed: a separator, a one-line hint
// and the prompt. Everything else scrolls above them, so output that arrives while you are typing (events,
// received frames) never lands in the middle of what you typed. It uses ANSI scroll regions, as tmux does, so
// it needs no library. Windows 10 and later are supported by turning on the console's VT mode.
//
// Threads: one background thread reads the input lines. printLine() may be called from any thread. nextLine()
// is for the one thread that runs the commands. The size of the terminal is read once, in start().

#include <condition_variable>
#include <deque>
#include <mutex>
#include <string>
#include <thread>

namespace console {

class Terminal {
public:
    Terminal() = default;
    ~Terminal();
    Terminal(const Terminal&) = delete;
    Terminal& operator=(const Terminal&) = delete;

    // Sets up the screen and starts reading input. `hint` is shown on the line above the prompt. Once.
    void start(const std::string& hint);

    // Puts the terminal back as it was. Safe to call twice. The input thread may still be waiting for the last
    // Enter; it is detached, and ends with the process.
    void stop();

    // Prints one line in the scrolling part. Any thread. Before start() it just prints.
    void printLine(const std::string& line);

    // Waits for the next line the user typed. False when the input has ended (end of file) or stop() was called.
    bool nextLine(std::string& line);

private:
    void redrawPromptLocked();
    void readLoop();

    std::mutex screenMutex_;  // every write to the screen
    std::mutex queueMutex_;   // queue_ and ended_
    std::condition_variable queueChanged_;
    std::deque<std::string> queue_;
    bool ended_ = false;
    bool started_ = false;
    int rows_ = 24;
    int columns_ = 80;
    int scrollBottom_ = 21;
    std::thread input_;
};

} // namespace console
