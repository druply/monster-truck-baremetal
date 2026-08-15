#include <boost/asio.hpp>
#include <thread>
#include <vector>
#include <iostream>

#include "modules/encoders/encoders.hpp"
#include "modules/motors/motors.hpp"
#include "modules/imu/imu.hpp"
#include "apis/state_estimation/state_estimation.hpp"
#include "apis/trajectory_follower/trajectory_follower.hpp"
#include "apis/trajectory_follower/trajectory_type.hpp"
#include "apis/publisher/Publisher.hpp"
#include <iomanip>
#include <algorithm>

//#include "modules/imu/imu_calibrator.hpp"
#include "modules/imu/axis_struct.hpp"

#include <chrono>

static Motors motors;
static Encoders encoders;
static Imu imu;
static Publisher publisher{encoders,imu,motors};


int main() {

    motors.init();
    encoders.init();
    imu.init(); 
    publisher.init();
    
    motors.setMotorsDirections(MotorsDirection_t::FORWARD);
    motors.setMotorsPwm(1.0);
    motors.setSteeringAngle(0.0);


        motors.setMotorsPwm(0.0);
        motors.deInit();
    return 0;

}