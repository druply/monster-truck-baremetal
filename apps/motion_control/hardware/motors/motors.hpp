#pragma once
#include "ModuleType.hpp"
#include "PWM.hpp"
#include <thread>


	// // boundary check for steering pwm
	// if(value > MAX_STERRING) {
	// 	value = MAX_STERRING;
	// }

	// if (value < -MAX_STERRING) {
	// 	value = -MAX_STERRING;
	// }

	// local_pwm = (int)(value * STEERING_M + STEERING_B);    

enum class MotorsDirection_t{
	FORWARD,
	BACKWARD
};

typedef struct  {
	int right;
	int left;
} MotorPwm_t ;

const int PWM_FREQ = 300;
const unsigned int PWM_PERIOD_NS = (unsigned int)(1'000'000'000LL / PWM_FREQ); // = 3333333 ns
const int STEERING_CHANNEL = 4;
const int REAR_MOTOR_CHANNEL = 1;
const int FRONT_MOTOR_CHANNEL = 0;
const int MOTOR_IN_A = 2;
const int MOTOR_IN_B = 3;



class Motors : public ModuleType{
	MotorPwm_t _motors_pwm;

	PWM motor_right{REAR_MOTOR_CHANNEL, PWM_PERIOD_NS}; 
	PWM motor_left{FRONT_MOTOR_CHANNEL, PWM_PERIOD_NS}; 
	PWM motor_ina{MOTOR_IN_A, PWM_PERIOD_NS}; 
	PWM motor_inb{MOTOR_IN_B, PWM_PERIOD_NS}; 
	PWM steering_motor{STEERING_CHANNEL, PWM_PERIOD_NS}; 

	//PCA9685Driver pwm{"/dev/i2c-1", 0x40};    
	public:
        Motors() noexcept;
        ~Motors();
        void init(void) noexcept override;
        void run(void) noexcept override;
        void deInit(void) noexcept override;
    	void setSteeringAngle(float value);
        void setMotorsDirections(MotorsDirection_t direction);
        void setMotorsPwm(float value);
};

