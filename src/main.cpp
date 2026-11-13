#include "robot/app/robot_factory.hpp"
#include "mocks/mock_encoder.hpp"
#include "mocks/mock_imu.hpp"
#include "mocks/mock_pwm.hpp"

#include <chrono>
#include <cstdio>
#include <memory>
#include <thread>

int main()
{
    robot::app::Hardware hardware{
        .imu = std::make_unique<MockImu>(),
        .left_encoder = std::make_unique<MockEncoder>(),
        .right_encoder = std::make_unique<MockEncoder>(),
        .rear_motor = std::make_unique<MockPWM>(),
        .front_motor = std::make_unique<MockPWM>()};

    auto robot_result = robot::app::make_robot_with_hardware(std::move(hardware));
    if (!robot_result)
    {
        std::fprintf(stderr, "Failed to initialize robot: %s\n", robot_result.error().message.c_str());
        return 1;
    }

    for (int iteration = 0; iteration < 10; ++iteration)
    {
        if (auto result = (*robot_result)->spin_once(); !result)
        {
            std::fprintf(stderr, "Robot loop failed: %s\n", result.error().message.c_str());
            return 1;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    if (auto result = (*robot_result)->shutdown(); !result)
    {
        std::fprintf(stderr, "Failed to stop motor outputs: %s\n", result.error().message.c_str());
        return 1;
    }

    return 0;
}