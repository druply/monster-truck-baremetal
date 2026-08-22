#include "motors.hpp"

#include <iostream>

Motors::Motors() noexcept
{
    // Constructor implementation
}

Motors::~Motors()
{
}

void Motors::init(void) noexcept
{
}
void Motors::run(void) noexcept
{
}

void Motors::setSteeringAngle(float value)
{
    _steering_angle = value;
    if (value > MAX_STERRING)
    {
        value = MAX_STERRING;
    }

    if (value < -MAX_STERRING)
    {
        value = -MAX_STERRING;
    }
    // caluclate pwm for pca9685
    float local_pwm = (value * STEERING_M + STEERING_B);
    // std::cout << "steering pwm: "<< local_pwm << std::endl;
    //  convert to duty cycle
    float duty = (static_cast<float>(local_pwm) / static_cast<float>(4096));
    // set duty cycle
    steering_motor.setPWM(duty);
    // pwm.setPWM(STEERING_CHANNEL, 0, local_pwm);
}

float Motors::getSteeringAngle(void)
{
    return _steering_angle;
}

MotorPwm_t Motors::getMotorsPwm(void)
{
    return _motors_pwm;
}

void Motors::setMotorsDirections(MotorsDirection_t direction)
{

    if (direction == MotorsDirection_t::FORWARD)
    {
        // std::cout << "direction forward" << std::endl;
        motor_ina.turnOn();
        motor_inb.turnOff();
    }
    else
    {
        motor_inb.turnOn();
        motor_ina.turnOff();
    }
}
void Motors::setMotorsPwm(float value)
{
    // std::cout << "motors pwm: "<< value << std::endl;
    motor_right.setPWM(value);
    motor_left.setPWM(value);

    _motors_pwm.right = static_cast<int>(value * 100);
    _motors_pwm.left = static_cast<int>(value * 100);
}

void Motors::deInit(void) noexcept
{
    motor_right.setPWM(0.0f);
    motor_left.setPWM(0.0f);
    // setSteeringAngle(0.0f); could represent safety issue
}