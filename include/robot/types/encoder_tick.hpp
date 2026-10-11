// encoder_tick.hpp
#pragma once
#include <cstdint>
#include "robot/types/imu_sample.hpp"  // Timestamp

namespace robot {
    struct EncoderTick {
        Timestamp stamp{};
        std::int64_t ticks{};   // cumulative, never wraps
    };
}