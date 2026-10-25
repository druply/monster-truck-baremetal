// include/robot/app/robot_factory.hpp
#pragma once
#include <memory>
#include "robot/app/robot.hpp"
#include "robot/app/hardware.hpp"
#include "robot/config/robot_config.hpp"

namespace robot::app
{

    // Builds a Robot with real hardware. Reads config from a file / env / params.
    // Throws or returns Result on construction failure — but once it returns,
    // the Robot is guaranteed initialized.
    Result<std::unique_ptr<Robot>> make_robot_from_config(const std::string &config_path);

    // Builds a Robot with mock hardware. Used in CI and on developer laptops.
    std::unique_ptr<Robot> make_robot_with_hardware(Hardware hw, robot::config::RobotConfig cfg);

} // namespace robot::app