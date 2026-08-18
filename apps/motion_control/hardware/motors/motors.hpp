#pragma once
#include "ModuleType.hpp"
#include "motors_types.hpp"
#include "IMotors.hpp"
#include <memory>

#include <thread>


	// // boundary check for steering pwm
	// if(value > MAX_STERRING) {
	// 	value = MAX_STERRING;
	// }

	// if (value < -MAX_STERRING) {
	// 	value = -MAX_STERRING;
	// }

	// local_pwm = (int)(value * STEERING_M + STEERING_B);    




class Motors : public IMotors, public ModuleType{
	
	struct Impl;
	std::unique_ptr<Impl> pimpl;


	//PCA9685Driver pwm{"/dev/i2c-1", 0x40};    
	public:
        Motors() noexcept;
        ~Motors();
        void init(void) noexcept override;
        void run(void) noexcept override;
        void deInit(void) noexcept override;
    	void setSteeringAngle(float value) override;
        void setMotorsDirections(MotorsDirection_t direction) override;
        void setMotorsPwm(float value) override;
};

