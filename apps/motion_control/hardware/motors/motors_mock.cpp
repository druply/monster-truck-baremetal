#include "motors.hpp"
#include <iostream>
#include "PWM.hpp"

const float STEERING_M	=		9.55;
const float STEERING_B		=	1860.5;

const float    MAX_STERRING     = 32.0;

const int PWM_FREQ = 300;
const unsigned int PWM_PERIOD_NS = (unsigned int)(1'000'000'000LL / PWM_FREQ); // = 3333333 ns
const int STEERING_CHANNEL = 4;
const int REAR_MOTOR_CHANNEL = 1;
const int FRONT_MOTOR_CHANNEL = 0;
const int MOTOR_IN_A = 2;
const int MOTOR_IN_B = 3;


struct Motors::Impl {
    // Mock implementation is intentionally minimal.
        PWM motor_right{REAR_MOTOR_CHANNEL, PWM_PERIOD_NS}; 
	    PWM motor_left{FRONT_MOTOR_CHANNEL, PWM_PERIOD_NS}; 
        PWM motor_ina{MOTOR_IN_A, PWM_PERIOD_NS}; 
        PWM motor_inb{MOTOR_IN_B, PWM_PERIOD_NS}; 
        PWM steering_motor{STEERING_CHANNEL, PWM_PERIOD_NS}; 
        MotorPwm_t _motors_pwm;
            Impl(){

 
    }
};

Motors::Motors() noexcept : pimpl(std::make_unique<Impl>()) {

}

Motors::~Motors() = default;

void Motors::init(void) noexcept {
    std::cout << "[Motors] init" << std::endl;
}
void Motors::run(void) noexcept {
std::cout << "[Motors] run" << std::endl;
}

void Motors::deInit(void) noexcept {
std::cout << "[Motors] deInit" << std::endl;
}
void Motors::setSteeringAngle(float value) {
std::cout << "[Motors] setSteeringAngle: "<< value << std::endl;
}
void Motors::setMotorsDirections(MotorsDirection_t direction) {
std::cout << "[Motors] setMotorsDirections" << std::endl;
}
void Motors::setMotorsPwm(float value){
    std::cout << "[Motors] setMotorsPwm"<< value << std::endl;
}