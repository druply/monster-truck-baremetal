// i_clock.hpp  -- critical for testability
#pragma once
#include "robot/hal/i_clock.hpp"
#include <chrono>

namespace utils {
    class Clock: public robot::hal::IClock {
    public:
        [[nodiscard]] robot::Timestamp monotonic_time_ms() const noexcept override {
            return static_cast<robot::Timestamp>(
                std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::steady_clock::now().time_since_epoch()).count());
        }
    };
}