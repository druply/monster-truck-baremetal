#pragma once

#include "robot/hal/i_pwm.hpp"

class MockPWM final : public robot::hal::IPWM
{
    bool m_healthy{false};

public:
    robot::PWMCommand last{};

    robot::Result<void> initialize() override
    {
        m_healthy = true;
        last = {};
        return {};
    }

    robot::Result<void> set(const robot::PWMCommand& command) override
    {
        if (!m_healthy)
        {
            return std::unexpected(robot::Error{
                robot::Module::Motor,
                robot::ErrorCode::NotInitialized,
                "Mock PWM is not initialized"});
        }
        last = command;
        return {};
    }

    robot::Result<void> turnOn() override
    {
        return set(robot::PWMCommand{.duty = 1.0});
    }

    robot::Result<void> turnOff() override
    {
        return set(robot::PWMCommand{.duty = 0.0});
    }
};
