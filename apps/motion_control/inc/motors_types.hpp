#pragma once

enum class MotorsDirection_t{
	FORWARD,
	BACKWARD
};

typedef struct  {
	int right;
	int left;
} MotorPwm_t ;
