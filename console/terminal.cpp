#include "terminal.hpp"

#include <cstdio>
#include <iostream>

#if defined(_WIN32)
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#else
#include <sys/ioctl.h>
#include <unistd.h>
#endif

namespace console {

namespace {

#if defined(_WIN32)
HANDLE g_stdout = nullptr;
DWORD g_originalMode = 0;
#endif

// Reads the size of the terminal. Keeps the defaults if it cannot be read.
void querySize(int& rows, int& columns) {
#if defined(_WIN32)
    CONSOLE_SCREEN_BUFFER_INFO info;
    if (g_stdout != nullptr && GetConsoleScreenBufferInfo(g_stdout, &info)) {
        const int r = info.srWindow.Bottom - info.srWindow.Top + 1;
        const int c = info.srWindow.Right - info.srWindow.Left + 1;
        if (r > 4) rows = r;
        if (c > 10) columns = c;
    }
#else
    struct winsize ws;
    if (ioctl(STDOUT_FILENO, TIOCGWINSZ, &ws) == 0) {
        if (ws.ws_row > 4) rows = ws.ws_row;
        if (ws.ws_col > 10) columns = ws.ws_col;
    }
#endif
}

void enableVt() {
#if defined(_WIN32)
    // POSIX terminals understand escape sequences. A Windows console needs VT mode turned on, or every sequence
    // below would be printed as characters.
    g_stdout = GetStdHandle(STD_OUTPUT_HANDLE);
    if (g_stdout != INVALID_HANDLE_VALUE && g_stdout != nullptr && GetConsoleMode(g_stdout, &g_originalMode)) {
        SetConsoleMode(g_stdout, g_originalMode | ENABLE_VIRTUAL_TERMINAL_PROCESSING);
    }
#endif
}

void restoreVt() {
#if defined(_WIN32)
    if (g_stdout != nullptr && g_stdout != INVALID_HANDLE_VALUE) {
        SetConsoleMode(g_stdout, g_originalMode);
    }
#endif
}

} // namespace

Terminal::~Terminal() {
    stop();
}

void Terminal::start(const std::string& hint) {
    enableVt();
    querySize(rows_, columns_);
    // Three rows below the scrolling part: separator, hint, prompt.
    scrollBottom_ = rows_ - 3 < 1 ? 1 : rows_ - 3;

    {
        std::lock_guard<std::mutex> lock(screenMutex_);
        // ESC[1;Nr limits scrolling to rows 1..N. That is the split. ESC[2J clears, ESC[H homes.
        std::printf("\x1b[1;%dr\x1b[2J\x1b[H", scrollBottom_);
        std::printf("\x1b[%d;1H%s", rows_ - 2, std::string(static_cast<std::size_t>(columns_), '-').c_str());
        std::printf("\x1b[%d;1H%.*s", rows_ - 1, columns_, hint.c_str());
        redrawPromptLocked();
        started_ = true;
    }
    input_ = std::thread([this] { readLoop(); });
}

void Terminal::stop() {
    {
        std::lock_guard<std::mutex> lock(queueMutex_);
        ended_ = true;
    }
    queueChanged_.notify_all();

    if (input_.joinable()) {
        // Nothing portable makes a blocked getline() return. The thread ends with the process.
        input_.detach();
    }
    std::lock_guard<std::mutex> lock(screenMutex_);
    if (started_) {
        started_ = false;
        std::printf("\x1b[1;%dr\x1b[%d;1H\n", rows_, rows_);  // the whole screen scrolls again
        std::fflush(stdout);
        restoreVt();
    }
}

void Terminal::redrawPromptLocked() {
    std::printf("\x1b[%d;1H\x1b[K> ", rows_);
    std::fflush(stdout);
}

void Terminal::printLine(const std::string& line) {
    std::lock_guard<std::mutex> lock(screenMutex_);
    if (!started_) {
        std::printf("%s\n", line.c_str());
        std::fflush(stdout);
        return;
    }
    // Save the cursor (it is in the middle of what the user is typing), print at the bottom of the scrolling
    // part, restore. ESC[s and ESC[u are used, not ESC 7 and ESC 8: "\x1b7" would be read by C++ as one
    // out-of-range hex escape.
    std::printf("\x1b[s\x1b[%d;1H%s\n\x1b[u", scrollBottom_, line.c_str());
    std::fflush(stdout);
}

bool Terminal::nextLine(std::string& line) {
    std::unique_lock<std::mutex> lock(queueMutex_);
    queueChanged_.wait(lock, [this] { return !queue_.empty() || ended_; });
    if (queue_.empty()) {
        return false;
    }
    line = std::move(queue_.front());
    queue_.pop_front();
    return true;
}

void Terminal::readLoop() {
    std::string line;
    while (std::getline(std::cin, line)) {
        {
            std::lock_guard<std::mutex> lock(queueMutex_);
            queue_.push_back(line);
        }
        queueChanged_.notify_one();
        std::lock_guard<std::mutex> lock(screenMutex_);
        if (started_) {
            redrawPromptLocked();  // the Enter may have scrolled the screen: put the prompt back
        }
    }
    {
        std::lock_guard<std::mutex> lock(queueMutex_);
        ended_ = true;
    }
    queueChanged_.notify_all();
}

} // namespace console
