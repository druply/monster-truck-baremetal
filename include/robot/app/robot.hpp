#pragma once

#include "robot/app/hardware.hpp"
#include "robot/core/odometry.hpp"
#include "robot/types/imu_sample.hpp"
#include "robot/types/pwm_command.hpp"
#include "robot/types/result.hpp"

namespace robot::app
{
    enum class MotorOutput
    {
        Rear,
        Front
    };

    struct RobotState
    {
        EncoderTick left_encoder{};
        EncoderTick right_encoder{};
        ImuSample imu{};
        core::OdometryState odometry{};
    };

    class Robot
    {
        Hardware m_hw;
        core::Odometry m_odometry;
        RobotState m_state{};
        bool m_rear_motor_initialized{false};
        bool m_front_motor_initialized{false};
        bool m_initialized{false};

    public:
        explicit Robot(Hardware hardware);
        ~Robot();
        Robot(const Robot&) = delete;
        Robot& operator=(const Robot&) = delete;

        [[nodiscard]] Result<void> initialize();
        [[nodiscard]] Result<void> spin_once();
        [[nodiscard]] Result<void> set_motor_output(MotorOutput output, const PWMCommand& command);
        [[nodiscard]] Result<void> shutdown();
        [[nodiscard]] const RobotState& state() const noexcept { return m_state; }
    };
} // namespace robot::app
