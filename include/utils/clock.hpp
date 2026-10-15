// i_clock.hpp  -- critical for testability
#pragma once
#include "robot/hal/i_clock.hpp"

namespace utils {
    class Clock: public robot::hal::IClock {
    public:
        [[nodiscard]] robot::Timestamp monotonic_time_ms() const noexcept override;
    };
}