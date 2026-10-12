#include "robot/robot.hpp"

#include <print>

namespace robot
{
Result<void> Robot::initialize() {

    std::printf("Initializing robot...\n");
    m_odometry->step();
     m_odometry->step();
    return Result<void>{};
 }
}