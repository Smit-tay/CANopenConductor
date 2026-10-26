#pragma once

#include <chrono>

namespace canopen_conductor {

// Tag clock for host-supplied time. The library never calls now() on it;
// the host passes the current time into every call.
struct HostClock {
    using duration = std::chrono::microseconds;
    using rep = duration::rep;
    using period = duration::period;
    using time_point = std::chrono::time_point<HostClock>;
    static constexpr bool is_steady = true;
};

using Duration = HostClock::duration;
using TimePoint = HostClock::time_point;

} // namespace canopen_conductor
