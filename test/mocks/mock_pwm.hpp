// test/mocks/mock_pwm.hpp
#include "robot/hal/i_pwm.hpp"
#include "robot/config/pwm_config.hpp"
#include <cstdio>
#include <string>

class MockPWM : public robot::hal::IPWM
{
    robot::config::PWMConfig m_config;
    std::string m_duty_path;

public:
    explicit MockPWM(const robot::config::PWMConfig &config) : m_config(config) {}

    robot::Result<void> initialize() override
    {
        std::printf("MockPWM initialized\n");
        m_duty_path = m_config.duty_path + std::to_string(m_config.channel) + "/duty_cycle";
        std::printf("duty_path: %s\n", m_duty_path.c_str());
        return {};
    }
    robot::Result<void> set(const robot::PWMCommand &cmd) override
    {
        last_ = cmd;
        std::printf("MockPWM set to %f\n", last_.duty);
        return {};
    }
    robot::Result<void> turnOn() override
    {
        std::printf("MockPWM turned on\n");
        return {};
    }

    robot::Result<void> turnOff() override
    {
        last_ = {};
        std::printf("MockPWM turned off\n");
        return {};
    }

    robot::PWMCommand last_{};

};