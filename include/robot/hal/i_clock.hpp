// i_clock.hpp  -- critical for testability
#pragma once
#include "robot/types/imu_sample.hpp"

namespace robot::hal {
    class IClock {
    public:
        virtual ~IClock() = default;
        [[nodiscard]] virtual Timestamp monotonic_time_ms() const noexcept = 0;
    };
}