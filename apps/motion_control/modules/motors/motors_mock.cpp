#include "motors.hpp"
#include <iostream>

const float STEERING_M	=		9.55;
const float STEERING_B		=	1860.5;

const float    MAX_STERRING     = 32.0;



Motors::Motors() noexcept {

}

Motors::~Motors() {

}

void Motors::init(void) noexcept {
    std::cout << "[Motors] init" << std::endl;
}
void Motors::run(void) noexcept {

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