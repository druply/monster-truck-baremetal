#pragma once

#include <memory>
#include "robot/app/hardware.hpp"
#include "robot/app/robot.hpp"

namespace robot::app
{
    [[nodiscard]] Result<std::unique_ptr<Robot>> make_robot_with_hardware(Hardware hardware);
} // namespace robot::app
