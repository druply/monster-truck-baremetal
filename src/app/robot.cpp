#include "robot/app/robot.hpp"

#include <cmath>
#include <cstdio>
#include <memory>
#include <optional>
#include <utility>

namespace robot::app
{
namespace
{
Error missing_device_error()
{
    return Error{
        Module::Runtime,
        ErrorCode::InvalidArgument,
        "Robot hardware is incomplete"};
}

void append_shutdown_error(Error& error, const Error& shutdown_error)
{
    error.message += "; safe motor shutdown also failed: " + shutdown_error.message;
}
} // namespace

Robot::Robot(Hardware hardware) : m_hw(std::move(hardware)) {}

Robot::~Robot()
{
    if (auto result = shutdown(); !result)
    {
        std::fprintf(stderr, "Robot shutdown failed: %s\n", result.error().message.c_str());
    }
}

Result<void> Robot::initialize()
{
    if (m_initialized)
    {
        return {};
    }
    if (!m_hw.imu || !m_hw.left_encoder || !m_hw.right_encoder ||
        !m_hw.rear_motor || !m_hw.front_motor)
    {
        return std::unexpected(missing_device_error());
    }

    if (auto result = m_hw.left_encoder->initialize(); !result)
    {
        return result;
    }
    if (auto result = m_hw.right_encoder->initialize(); !result)
    {
        return result;
    }
    if (auto result = m_hw.imu->initialize(); !result)
    {
        return result;
    }
    if (auto result = m_hw.rear_motor->initialize(); !result)
    {
        return result;
    }
    m_rear_motor_initialized = true;

    if (auto result = m_hw.front_motor->initialize(); !result)
    {
        Error error = result.error();
        if (auto stop_result = shutdown(); !stop_result)
        {
            append_shutdown_error(error, stop_result.error());
        }
        return std::unexpected(std::move(error));
    }
    m_front_motor_initialized = true;

    m_odometry = core::Odometry{};
    m_state = {};
    m_initialized = true;
    return {};
}

Result<void> Robot::spin_once()
{
    if (!m_initialized)
    {
        return std::unexpected(Error{
            Module::Runtime,
            ErrorCode::NotInitialized,
            "Robot must be initialized before spinning"});
    }

    const auto fail_safe = [this](Error error) -> Result<void>
    {
        if (auto stop_result = shutdown(); !stop_result)
        {
            append_shutdown_error(error, stop_result.error());
        }
        return std::unexpected(std::move(error));
    };

    if (auto result = m_hw.left_encoder->run(); !result)
    {
        return fail_safe(result.error());
    }
    if (auto result = m_hw.right_encoder->run(); !result)
    {
        return fail_safe(result.error());
    }
    auto left_tick = m_hw.left_encoder->read();
    if (!left_tick)
    {
        return fail_safe(left_tick.error());
    }
    auto right_tick = m_hw.right_encoder->read();
    if (!right_tick)
    {
        return fail_safe(right_tick.error());
    }
    auto imu_sample = m_hw.imu->read();
    if (!imu_sample)
    {
        return fail_safe(imu_sample.error());
    }
    auto odometry = m_odometry.update(*right_tick, *left_tick);
    if (!odometry)
    {
        return fail_safe(odometry.error());
    }

    m_state.left_encoder = *left_tick;
    m_state.right_encoder = *right_tick;
    m_state.imu = *imu_sample;
    m_state.odometry = *odometry;
    return {};
}

Result<void> Robot::set_motor_output(MotorOutput output, const PWMCommand& command)
{
    if (!m_initialized)
    {
        return std::unexpected(Error{
            Module::Runtime,
            ErrorCode::NotInitialized,
            "Robot must be initialized before setting motor outputs"});
    }

    hal::IPWM* motor = nullptr;
    switch (output)
    {
    case MotorOutput::Rear:
        motor = m_hw.rear_motor.get();
        break;
    case MotorOutput::Front:
        motor = m_hw.front_motor.get();
        break;
    default:
        return std::unexpected(Error{
            Module::Motor,
            ErrorCode::InvalidArgument,
            "Invalid motor output selection"});
    }

    if (!std::isfinite(command.duty) || command.duty < 0.0 || command.duty > 1.0)
    {
        Error error{
            Module::Motor,
            ErrorCode::InvalidArgument,
            "PWM duty must be finite and in [0, 1]"};
        if (auto stop_result = shutdown(); !stop_result)
        {
            append_shutdown_error(error, stop_result.error());
        }
        return std::unexpected(std::move(error));
    }

    if (auto result = motor->set(command); !result)
    {
        Error error = result.error();
        if (auto stop_result = shutdown(); !stop_result)
        {
            append_shutdown_error(error, stop_result.error());
        }
        return std::unexpected(std::move(error));
    }
    return {};
}

Result<void> Robot::shutdown()
{
    m_initialized = false;
    std::optional<Error> first_error;

    if (m_front_motor_initialized)
    {
        auto result = m_hw.front_motor->turnOff();
        if (!result)
        {
            if (first_error)
            {
                first_error->message += "; front motor shutdown failed: " + result.error().message;
            }
            else
            {
                first_error = result.error();
            }
        }
        else
        {
            m_front_motor_initialized = false;
        }
    }
    if (m_rear_motor_initialized)
    {
        auto result = m_hw.rear_motor->turnOff();
        if (!result && first_error)
        {
            first_error->message += "; rear motor shutdown failed: " + result.error().message;
        }
        else if (!result)
        {
            first_error = result.error();
        }
        else if (result)
        {
            m_rear_motor_initialized = false;
        }
    }

    if (first_error)
    {
        return std::unexpected(std::move(*first_error));
    }
    return {};
}
} // namespace robot::app
