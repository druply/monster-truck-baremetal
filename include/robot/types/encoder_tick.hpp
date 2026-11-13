// encoder_tick.hpp
#pragma once
#include <cstdint>
#include "robot/types/timestamp.hpp"

namespace robot {
    using Tick = std::uint64_t; // cumulative, never wraps
    struct EncoderTick {
        Timestamp stamp{};
        Tick ticks{};   // cumulative, never wraps
    };
}