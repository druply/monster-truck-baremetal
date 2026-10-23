// src/app/robot_factory.cpp
#include "robot/app/robot_factory.hpp"
//#include "robot/config/config_loader.hpp"

#include "robot/drivers/mpu9250_imu.hpp"
#include "robot/drivers/lm393_encoder.hpp"
#include "robot/drivers/pca9685_motor.hpp"
//#include "robot/drivers/rplidar_driver.hpp"
//#include "robot/drivers/linux_clock.hpp"

namespace robot::app {

Result<std::unique_ptr<Robot>> make_robot_from_config(const std::string& /*path*/) {
    //auto cfg = config::load(path);          // parses YAML into plain structs
    RobotConfig cfg;
    //auto clock = std::make_shared<drivers::LinuxClock>();

    Hardware hw{
        // .imu = std::make_unique<drivers::Mpu9250Imu>(
        //     cfg.imu, clock),
        .left_encoder = std::make_unique<MockEncoder>(
            /*cfg.left_encoder*/),
        .right_encoder = std::make_unique<MockEncoder>(
            /*cfg.right_encoder*/),
        // .left_motor = std::make_unique<drivers::Pca9685Motor>(
        //     cfg.left_motor),
        // .right_motor = std::make_unique<drivers::Pca9685Motor>(
        //     cfg.right_motor),
        // .lidar = std::make_unique<drivers::RplidarDriver>(
        //     cfg.lidar),
        //.clock = clock,
    };

    auto robot = std::make_unique<Robot>(cfg.robot, std::move(hw));
    if (auto r = robot->initialize(); !r) return std::unexpected(r.error());
    return robot;
}

} // namespace robot::app