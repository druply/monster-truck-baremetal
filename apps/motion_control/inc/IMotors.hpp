#pragma once
#include "motors_types.hpp"

class IMotors {
public:
    virtual ~IMotors() = default;
    virtual void setSteeringAngle(float value) = 0;
    virtual void setMotorsDirections(MotorsDirection_t direction) = 0;
    virtual void setMotorsPwm(float value) = 0;
};
