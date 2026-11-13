// src/app/robot_factory.cpp
#include "robot/app/robot_factory.hpp"
// #include "robot/config/config_loader.hpp"
#include "mocks/mock_imu.hpp"
#include "mocks/mock_encoder.hpp"
#include "mocks/mock_pwm.hpp"

#include "robot/drivers/mpu9250_imu.hpp"
#include "robot/drivers/lm393_encoder.hpp"
// #include "robot/drivers/rplidar_driver.hpp"
// #include "robot/drivers/linux_clock.hpp"

namespace robot::app
{
    const int PWM_FREQ = 300;
    const unsigned int PWM_PERIOD_NS = static_cast<unsigned int>(1'000'000'000LL / PWM_FREQ); // = 3333333 ns

    Result<std::unique_ptr<Robot>> make_robot_from_config(const std::string & /*path*/)
    {
        // auto cfg = config::load(path);          // parses YAML into plain structs
        robot::config::RobotConfig cfg{
            .imu = "path/to/imu_config.yaml",
            .left_encoder = robot::config::EncoderConfig{
                .gpio_chip = "gpiochip0",
                .line_name = "encoder_left",
                .line = 18},
            .right_encoder = robot::config::EncoderConfig{.gpio_chip = "gpiochip0", .line_name = "encoder_right", .line = 17},
             .robot = "path/to/robot_config.yaml",
            .rear_motor_control_pin = robot::config::PWMConfig{.duty_path = "/sys/class/pwm/pwmchip0/pwm", .channel = 1, .max_period = PWM_PERIOD_NS},
            .front_motor_control_pin = robot::config::PWMConfig{.duty_path = "/sys/class/pwm/pwmchip0/pwm", .channel = 0, .max_period = PWM_PERIOD_NS}
           };
        // auto clock = std::make_shared<drivers::LinuxClock>();

        Hardware hw{
            .imu = std::make_unique<MockImu>(),
            // .imu = std::make_unique<drivers::Mpu9250Imu>(
            //     cfg.imu, clock),
            .left_encoder = std::make_unique<MockEncoder>(
                cfg.left_encoder),
            .right_encoder = std::make_unique<MockEncoder>(
                cfg.right_encoder),
            // .left_motor = std::make_unique<drivers::Pca9685Motor>(
            //     cfg.left_motor),
            .rear_motor = std::make_unique<MockPWM>(cfg.rear_motor_control_pin),
            .front_motor = std::make_unique<MockPWM>(cfg.front_motor_control_pin),
            // .lidar = std::make_unique<drivers::RplidarDriver>(
            //     cfg.lidar),
            //.clock = clock,
        };

        auto robot = std::make_unique<Robot>(cfg.robot, std::move(hw));

        if (auto r = robot->initialize(); !r)
            return std::unexpected(r.error());
        return robot;
    }

} // namespace robot::app