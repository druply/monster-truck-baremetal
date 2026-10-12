#include "robot/robot.hpp"
#include <print>

int main() {
    robot::Robot robot;

    robot::Result<void> result = robot.initialize();
    if (!result) {
        std::printf("Failed to initialize robot: %d\n", static_cast<int>(result.error()));
        return 1;
    }

    
    return 0;
}