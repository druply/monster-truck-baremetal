#pragma once
#include <string>
#include "robot/config/encoder_config.hpp"
#include "robot/config/pwm_config.hpp"
namespace robot::config
{

    struct RobotConfig
    {
        std::string imu;
        EncoderConfig left_encoder;
        EncoderConfig right_encoder;
        std::string robot;
        PWMConfig rear_motor_control_pin;
        PWMConfig front_motor_control_pin;
    };

} // namespace robot::config