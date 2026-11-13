#include "robot/app/robot.hpp"

#include <print>

namespace robot::app
{
    Result<void> Robot::initialize()
    {
        std::printf("Initializing robot...\n");
        robot::Result<void> res;
        res = m_hw.left_encoder->initialize();
        if (!res)
        {
            std::printf("Failed to initialize left encoder: %s\n", res.error().message.c_str());
            // return std::unexpected(res.error());
            return res;
        }
        res = m_hw.right_encoder->initialize();
        if (!res)
        {
            std::printf("Failed to initialize right encoder: %s\n", res.error().message.c_str());
            // return std::unexpected(res.error());
            return res;
        }
        m_encoder_thread = std::jthread([this](std::stop_token stoken)
                                        { encoder_loop(stoken); });
        // m_odometry->step();
        //  m_odometry->step();

        res = m_hw.imu->initialize();
        if (!res)
        {
            std::printf("Failed to initialize IMU: %s\n", res.error().message.c_str());
            return res;
        }

        res = m_hw.rear_motor->initialize();
        if (!res)
        {
            std::printf("Failed to initialize rear motor: %s\n", res.error().message.c_str());
            return res;
        }
        res = m_hw.front_motor->initialize();
        if (!res)
        {
            std::printf("Failed to initialize front motor: %s\n", res.error().message.c_str());
            return res;
        }

        return Result<void>{};
    }

    Result<bool> Robot::spin_once()
    {
        
        std::optional<robot::EncoderTick> left_tick = m_hw.left_encoder->read();
        
        std::optional<robot::EncoderTick> right_tick = m_hw.right_encoder->read();
        
        if (!left_tick.has_value() && !right_tick.has_value())
        {
            std::printf("Failed to get left and right encoder ticks\n");
            return std::unexpected(Error{
                Module::Encoder,
                ErrorCode::IoFailure,
                "Failed to get left and right encoder ticks"});
        }

        std::cout << "Left tick: " << left_tick.value().ticks << ", Right tick: " << right_tick.value().ticks << "\n";

        robot::Result<robot::ImuSample> imu_data = m_hw.imu->read();

        if (!imu_data)
        {
            std::printf("Failed to get IMU data\n");
            return std::unexpected(imu_data.error());
        }

        std::cout << "IMU accel: (" << imu_data.value().accel.x << ", " << imu_data.value().accel.y << ", " << imu_data.value().accel.z << "), "
                  << "gyro: (" << imu_data.value().gyro.x << ", " << imu_data.value().gyro.y << ", " << imu_data.value().gyro.z << "), "
                  << "mag: (" << imu_data.value().mag.x << ", " << imu_data.value().mag.y << ", " << imu_data.value().mag.z << ")\n";

        m_odometry.update(right_tick.value(), left_tick.value(), imu_data.value());

        m_demo.update(right_tick.value().ticks, left_tick.value().ticks);
        // m_hw.rear_motor->set(robot::PWMCommand{.duty = 0.5});
        // m_hw.front_motor->set(robot::PWMCommand{.duty = 0.5});
        // std::this_thread::sleep_for(std::chrono::milliseconds(100));

        return Result<bool>{true};
    }

    void Robot::encoder_loop(std::stop_token stoken)
    {
        while (!stoken.stop_requested())
        {
            m_hw.left_encoder->run();
            m_hw.right_encoder->run();
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    }
}