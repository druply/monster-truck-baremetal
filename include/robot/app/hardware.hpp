// include/robot/app/hardware.hpp
#pragma once
#include <memory>
#include "robot/hal/i_imu.hpp"
#include "robot/hal/i_encoder.hpp"
#include "robot/hal/i_motor.hpp"
#include "robot/hal/i_lidar.hpp"
#include "robot/hal/i_clock.hpp"

namespace robot::app
{

    struct Hardware
    {
        //std::unique_ptr<hal::IImu> imu;
        std::unique_ptr<hal::IEncoder> left_encoder;
        std::unique_ptr<hal::IEncoder> right_encoder;
        // std::unique_ptr<hal::IMotor> left_motor;
        // std::unique_ptr<hal::IMotor> right_motor;
        // std::unique_ptr<hal::ILidar> lidar;
        // std::shared_ptr<hal::IClock> clock; // shared: many things need time
    };

} // namespace robot::app