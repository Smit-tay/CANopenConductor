# CANopenConductor

An embeddable CANopen master library for C++20. You call it from your own
I/O loop. It starts no threads, reads no clock, and never touches the bus
itself.

## Features

- NMT master and heartbeat monitoring
- SDO client (expedited, segmented, block)
- PDO and multiplexed-PDO consumer
- SYNC and TIME producers
- LSS master
- EMCY consumer

## Design

- **Passive.** The host loop reads frames from the bus and hands them to
  `Master::onFrame()`, and calls `Master::tick()` as time passes.
  `Master::nextDeadline()` tells the loop how long it may wait.
- **One I/O thread.** Every call into a `Master` is made from the same
  thread. The library takes no locks.
- **Host-provided memory.** `Master::requiredBytes(config)` tells you how
  much memory a configuration needs. You allocate it (and lock it, if you run
  with real-time constraints) and pass it to `Master::create()`. All master
  state lives in that block; there is nothing to destroy.
- **Bring your own bus.** The library talks to the bus only through the
  `ICanBus` interface. `send()` must never block.
- **Callbacks** run on the I/O thread. Keep them short and non-blocking, and
  do not call into the `Master` from inside one.

## Building

Requires CMake 3.16 and a C++20 compiler.

```cmake
add_subdirectory(CANopenConductor)
target_link_libraries(my_app PRIVATE CANopenConductor::CANopenConductor)
```

There is one target. It contains the CANopen protocol code, which uses no OS
headers, and the connection code, which is compiled per platform:

| File | Platform |
|---|---|
| `link.hpp` | all: one CAN connection over whichever port fits the interface name |
| `session.hpp` | all: the life of a connection (connect until it opens, report a loss, reconnect on command) |
| `slcan_bus.hpp` | serial adapters speaking slcan: POSIX tty, Windows COM port (Windows is experimental) |
| `socketcan_bus.hpp` | Linux (SocketCAN) |
| `waker.hpp` | all: lets another thread interrupt a blocked `receive()` |

A project with its own `ICanBus` just does not use these headers.

CMake options:

| Option | Default | |
|---|---|---|
| `CANOPEN_CONDUCTOR_TESTS` | OFF | build the connection tests (Linux) |
| `CANOPEN_CONDUCTOR_CONSOLE` | OFF | build `canopen-console`, a program to try the connection by hand |
| `CANOPEN_CONDUCTOR_WERROR` | OFF | treat warnings as errors |

The library is built with `-fno-exceptions -fno-rtti` (`/EHs-c- /GR-` on
MSVC). These flags apply to the library only. Your code is unaffected.

## The connection

`Session` (on top of `Link`) is what a host uses to keep one CAN interface connected. It starts no thread: the
host calls `step()` in a loop on its own I/O thread, and gives the commands from any thread.

- `connect()` opens the interface and keeps trying until it opens. The first failure is reported once
  (`ConnectFailed`), later attempts are quiet. A success is reported (`Connected`).
- A lost interface is reported once (`Lost`). A `Session` never reconnects by itself.
- `reconnect()` starts a new connect command, but only if a connection is wanted and nothing is open or being
  tried. `disconnect()` closes the interface and stops trying.

`canopen-console <interface> [bitrate]` runs a `Session` the same way, with a small split-screen console
(`HELP`, `STATUS`, `CONNECT`, `RECONNECT`, `DISCONNECT`, `QUIT`).

## Use

```cpp
#include <canopen_conductor/master.hpp>
using namespace canopen_conductor;

MyBus bus;                       // your ICanBus, or Link / one of the ports
MasterConfig config;             // nodes, PDOs, callbacks, ...
// ... fill in config ...

const std::size_t bytes = Master::requiredBytes(config);
std::unique_ptr<std::max_align_t[]> memory(
    new std::max_align_t[(bytes + sizeof(std::max_align_t) - 1) / sizeof(std::max_align_t)]);
// lock the memory here if you need to (e.g. mlockall on Linux)

Master* master = Master::create(memory.get(), bytes, config, bus);

for (;;) {
    // hostNow() and timeoutUntil() are your helpers: a monotonic clock, and
    // the time remaining until master->nextDeadline(now).
    TimePoint now = hostNow();
    CanFrame frame;
    if (bus.receive(frame, timeoutUntil(master->nextDeadline(now), now)) == RxStatus::Frame)
        master->onFrame(frame, hostNow());
    master->tick(hostNow());
}
```

`Master::create()` returns `nullptr` if the configuration is invalid or the
memory is too small. Requests such as `sdoUpload()`, `sdoDownload()` and
`nmtCommand()` are made from the same loop. Results come back through the
callbacks in `MasterConfig`.

## Layout

| Directory | Contents |
|---|---|
| `include/canopen_conductor/` | public headers |
| `src/` | library sources |
| `test/` | connection test |

## License

MIT. See `LICENSE`.

CANopen® is a registered trademark of CAN in Automation e.V.
