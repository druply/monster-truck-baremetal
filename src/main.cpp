#include "robot/app/robot_factory.hpp"
#include <memory>
#include <print>

int main() {
    // robot::Robot robot;

    // robot::Result<void> result = robot.initialize();
    // if (!result) {
    //     std::printf("Failed to initialize robot: \n");
    //     return 1;
    // }

    robot::Result<std::unique_ptr<robot::app::Robot>> robot_result = robot::app::make_robot_from_config("path/to/config.yaml");
    
    if (!robot_result) {
        std::printf("Failed to create robot from config: %s\n", robot_result.error().message.c_str());
        return 1;
    }

    //robot_result.value()->initialize();

    return 0;
}