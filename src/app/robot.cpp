#include "robot/app/robot.hpp"

#include <print>

namespace robot::app
{
Result<void> Robot::initialize() {

    std::printf("Initializing robot...\n");
    // m_odometry->step();
    //  m_odometry->step();
    return Result<void>{};
 }
}