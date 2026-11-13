// i_pwm.hpp
#pragma once
#include "robot/types/pwm_command.hpp"
#include "robot/types/result.hpp"

namespace robot::hal
{
    class IPWM
    {
    public:
        virtual ~IPWM() = default;
        virtual Result<void> initialize() = 0;
        virtual Result<void> set(const PWMCommand &cmd) = 0;
        virtual Result<void> turnOn(void) = 0;
        virtual Result<void> turnOff(void) = 0;
    };
}