// test/mocks/mock_motor.hpp
#include "robot/hal/i_motor.hpp"
#include "robot/config/motor_config.hpp"
#include <cstdio>
#include <string>

class MockMotor : public robot::hal::IMotor
{
    robot::config::MotorConfig m_config;
    std::string m_duty_path;

public:
    explicit MockMotor(const robot::config::MotorConfig &config) : m_config(config) {}

    robot::Result<void> initialize() override
    {
        std::printf("MockMotor initialized\n");
        m_duty_path = m_config.duty_path + std::to_string(m_config.channel) + "/duty_cycle";
        std::printf("duty_path: %s\n", m_duty_path.c_str());
        return {};
    }
    robot::Result<void> set(const robot::MotorCommand &cmd) override
    {
        last_ = cmd;
        std::printf("MockMotor set to %f\n", last_.duty);
        return {};
    }
    robot::Result<void> stop() override
    {
        last_ = {};
        std::printf("MockMotor stopped\n");
        return {};
    }
    bool is_healthy() const noexcept override { return healthy_; }

    robot::MotorCommand last_{};
    bool healthy_{true};
};