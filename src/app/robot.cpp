#include "robot/app/robot.hpp"

#include <print>

namespace robot::app
{
Result<void> Robot::initialize() {

    std::printf("Initializing robot...\n");
    robot::Result<void>  res ;
    res = m_hw.left_encoder->initialize();
    if (!res) {
        std::printf("Failed to initialize left encoder: %s\n", res.error().message.c_str());
        return std::unexpected(res.error());
    }
    res = m_hw.right_encoder->initialize();
    if (!res) {
        std::printf("Failed to initialize right encoder: %s\n", res.error().message.c_str());
        return std::unexpected(res.error());
    }
    // m_odometry->step();
    //  m_odometry->step();
    return Result<void>{};
 }
}