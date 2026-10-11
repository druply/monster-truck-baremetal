// i_motor.hpp
#pragma once
#include "robot/types/motor_command.hpp"
#include "robot/types/result.hpp"

namespace robot::hal {
    class IMotor {
    public:
        virtual ~IMotor() = default;
        virtual Result<void> initialize() = 0;
        virtual Result<void> set(const MotorCommand& cmd) = 0;
        virtual Result<void> stop() = 0;   // safe-state = coast/brake
        [[nodiscard]] virtual bool is_healthy() const noexcept = 0;
    };
}