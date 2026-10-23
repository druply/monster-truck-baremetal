#pragma once
#include <string>

namespace robot::app
{

    struct RobotConfig
    {
        std::string imu;
        std::string left_encoder;
        std::string right_encoder;
        std::string robot;
    };

} // namespace robot::app