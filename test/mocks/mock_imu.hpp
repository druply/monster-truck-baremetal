#pragma once

#include "robot/hal/i_imu.hpp"

class MockImu final : public robot::hal::IImu
{
    bool m_healthy{false};

public:
    bool fail_read{false};
    robot::ImuSample next{
        0,
        {0.5, 0.3, 0.7},
        {0.1, 0.2, 0.21},
        {0.0, 0.0, 0.0},
        0.0};

    robot::Result<void> initialize() override
    {
        m_healthy = true;
        return {};
    }

    robot::Result<robot::ImuSample> read() override
    {
        if (!m_healthy)
        {
            return std::unexpected(robot::Error{
                robot::Module::Imu,
                robot::ErrorCode::NotInitialized,
                "Mock IMU is not initialized"});
        }
        if (fail_read)
        {
            return std::unexpected(robot::Error{
                robot::Module::Imu,
                robot::ErrorCode::IoFailure,
                "Mock IMU read failed"});
        }
        return next;
    }

    [[nodiscard]] bool is_healthy() const noexcept override { return m_healthy; }
};
