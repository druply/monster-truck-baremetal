#pragma once
#include <string>
#include "robot/config/encoder_config.hpp"

namespace robot::config
{

    struct RobotConfig
    {
        std::string imu;
        EncoderConfig left_encoder;
        EncoderConfig right_encoder;
        std::string robot;
    };

} // namespace robot::app