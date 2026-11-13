// i_encoder.hpp
#pragma once
#include "robot/types/encoder_tick.hpp"
#include "robot/types/result.hpp"

namespace robot::hal {
    class IEncoder {
    public:
        virtual ~IEncoder() = default;
        virtual Result<void> initialize() = 0;
        [[nodiscard]] virtual Result<EncoderTick> read() const = 0;
        virtual Result<void> run() = 0;
        [[nodiscard]] virtual bool is_healthy() const noexcept = 0;
    };
}