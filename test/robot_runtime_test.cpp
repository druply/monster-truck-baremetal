#include "robot/app/robot_factory.hpp"
#include "robot/core/odometry.hpp"
#include "robot/drivers/pwm.hpp"
#include "mocks/mock_encoder.hpp"
#include "mocks/mock_imu.hpp"
#include "mocks/mock_pwm.hpp"

#include <cmath>
#include <cstdlib>
#include <memory>

namespace
{
template <typename T>
void check(T&& condition)
{
    if (!condition)
    {
        std::abort();
    }
}

void test_robot_lifecycle_and_state()
{
    auto rear_motor = std::make_unique<MockPWM>();
    auto front_motor = std::make_unique<MockPWM>();
    auto imu_owner = std::make_unique<MockImu>();
    MockPWM* rear_motor_ptr = rear_motor.get();
    MockPWM* front_motor_ptr = front_motor.get();

    robot::app::Hardware hardware{
        .imu = std::move(imu_owner),
        .left_encoder = std::make_unique<MockEncoder>(),
        .right_encoder = std::make_unique<MockEncoder>(),
        .rear_motor = std::move(rear_motor),
        .front_motor = std::move(front_motor)};

    auto robot_result = robot::app::make_robot_with_hardware(std::move(hardware));
    check(robot_result);
    auto& robot = **robot_result;

    check(robot.set_motor_output(robot::app::MotorOutput::Rear, robot::PWMCommand{.duty = 0.4}));
    check(rear_motor_ptr->last.duty == 0.4);
    check(robot.spin_once());
    check(robot.state().left_encoder.ticks == 1);
    check(robot.state().right_encoder.ticks == 1);
    check(std::isfinite(robot.state().odometry.left_distance_m));
    check(std::isfinite(robot.state().odometry.right_distance_m));
    check(!robot.set_motor_output(robot::app::MotorOutput::Rear, robot::PWMCommand{.duty = 1.1}));
    check(rear_motor_ptr->last.duty == 0.0);
    check(front_motor_ptr->last.duty == 0.0);

    check(robot.shutdown());
    check(rear_motor_ptr->last.duty == 0.0);
    check(front_motor_ptr->last.duty == 0.0);
    check(!robot.spin_once());
    check(robot.initialize());
    check(robot.spin_once());
    check(robot.state().left_encoder.ticks == 1);
    check(robot.shutdown());
}

void test_sensor_failure_stops_outputs()
{
    auto imu = std::make_unique<MockImu>();
    MockImu* imu_ptr = imu.get();
    auto rear_motor = std::make_unique<MockPWM>();
    auto front_motor = std::make_unique<MockPWM>();
    MockPWM* rear_motor_ptr = rear_motor.get();
    MockPWM* front_motor_ptr = front_motor.get();
    robot::app::Hardware hardware{
        .imu = std::move(imu),
        .left_encoder = std::make_unique<MockEncoder>(),
        .right_encoder = std::make_unique<MockEncoder>(),
        .rear_motor = std::move(rear_motor),
        .front_motor = std::move(front_motor)};

    auto robot_result = robot::app::make_robot_with_hardware(std::move(hardware));
    check(robot_result);
    imu_ptr->fail_read = true;
    check(!(*robot_result)->spin_once());
    check(rear_motor_ptr->last.duty == 0.0);
    check(front_motor_ptr->last.duty == 0.0);
    check(!(*robot_result)->spin_once());
}

void test_odometry_guards()
{
    robot::core::Odometry odometry;
    const robot::EncoderTick first_right{100, 10};
    const robot::EncoderTick first_left{100, 10};
    auto first = odometry.update(first_right, first_left);
    check(first);
    check(first->right_velocity_mps == 0.0);

    const robot::EncoderTick second_right{100, 11};
    const robot::EncoderTick second_left{200, 11};
    auto second = odometry.update(second_right, second_left);
    check(second);
    check(second->right_velocity_mps == 0.0);
    check(std::isfinite(second->left_velocity_mps));
    check(second->left_velocity_mps > 0.0);

    const robot::EncoderTick backwards_right{300, 9};
    const robot::EncoderTick backwards_left{300, 12};
    check(!odometry.update(backwards_right, backwards_left));
}

void test_pwm_configuration_validation()
{
    robot::drivers::Pca9685Pwm pwm(robot::config::PWMConfig{.channel = 16});
    auto result = pwm.initialize();
    check(!result);
    check(result.error().code == robot::ErrorCode::InvalidArgument);
    check(!pwm.set(robot::PWMCommand{.duty = 0.5}));
}
} // namespace

int main()
{
    test_robot_lifecycle_and_state();
    test_sensor_failure_stops_outputs();
    test_odometry_guards();
    test_pwm_configuration_validation();
}
