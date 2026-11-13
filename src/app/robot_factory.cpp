#include "robot/app/robot_factory.hpp"

#include <memory>
#include <utility>

namespace robot::app
{
Result<std::unique_ptr<Robot>> make_robot_with_hardware(Hardware hardware)
{
    auto robot = std::make_unique<Robot>(std::move(hardware));
    if (auto result = robot->initialize(); !result)
    {
        return std::unexpected(result.error());
    }
    return robot;
}
} // namespace robot::app
