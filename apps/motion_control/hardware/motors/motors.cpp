#include "motors.hpp"

#include "PWM.hpp"
#include <iostream>

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
    // Implementation details can be added here if needed
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
    // Constructor implementation

}

Motors::~Motors() {

}

void Motors::init(void) noexcept {

}
void Motors::run(void) noexcept {

}

void Motors::setSteeringAngle(float value) {

    if(value > MAX_STERRING) {
	    value = MAX_STERRING;
	}

	if (value < -MAX_STERRING) {
	    value = -MAX_STERRING;
	}
    // caluclate pwm for pca9685
	float local_pwm = (value * STEERING_M + STEERING_B);    
    //std::cout << "steering pwm: "<< local_pwm << std::endl;
    // convert to duty cycle
    float duty = (static_cast<float>(local_pwm)/static_cast<float>(4096));
    // set duty cycle
    pimpl->steering_motor.setPWM(duty);
    //pwm.setPWM(STEERING_CHANNEL, 0, local_pwm);
}

void Motors::setMotorsDirections(MotorsDirection_t direction) {
    
    if (direction == MotorsDirection_t::FORWARD) {
         //std::cout << "direction forward" << std::endl;
        pimpl->motor_ina.turnOn();
        pimpl->motor_inb.turnOff();
       
    }
    else {
        pimpl->motor_inb.turnOn();
        pimpl->motor_ina.turnOff();
    }
}
void Motors::setMotorsPwm(float value){
    //std::cout << "motors pwm: "<< value << std::endl;
    pimpl->motor_right.setPWM(value);
    pimpl->motor_left.setPWM(value);
   
}


void Motors::deInit(void) noexcept {
    pimpl->motor_right.setPWM(0.0f);
    pimpl->motor_left.setPWM(0.0f);
    //setSteeringAngle(0.0f); could represent safety issue
}