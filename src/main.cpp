#include "robot/app/robot_factory.hpp"
#include <chrono>
#include <memory>
#include <print>
#include <thread>

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

     std::this_thread::sleep_for(std::chrono::milliseconds(10));
    for (int i = 0; i < 10; ++i) {
        robot::Result<bool> result = robot_result.value()->spin_once();
        if (!result) {
            std::printf("Failed to spin robot: %s\n", result.error().message.c_str());
            return 1;
        }
    }
       
    //robot_result.value()->initialize();

    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    return 0;
}