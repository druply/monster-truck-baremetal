#include <boost/asio.hpp>
#include <thread>
#include <vector>
#include <iostream>

#include "hardware/encoders/encoders.hpp"
#include "hardware/motors/motors.hpp"
#include "hardware/imu/imu.hpp"
#include "apis/state_estimation/state_estimation.hpp"
#include "apis/trajectory_follower/trajectory_follower.hpp"
#include "apis/trajectory_follower/trajectory_type.hpp"
#include "apis/publisher/Publisher.hpp"
#include <iomanip>
#include <algorithm>
#include <array>

//#include "hardware/imu/imu_calibrator.hpp"
#include "imu_types.hpp"

#include <chrono>

static Motors motors;
static Encoders encoders;
static Imu imu;
static Publisher publisher{encoders,imu,motors};


int main() {
    // Initialize modules
    motors.init();
    encoders.init();
    imu.init();
    publisher.init();
    
    motors.setMotorsDirections(MotorsDirection_t::FORWARD);
    motors.setMotorsPwm(1.0);
    motors.setSteeringAngle(0.0);

    // Run modules
    motors.run();
    encoders.run();
    imu.run();
    publisher.run();


    // deinit modules
    motors.setMotorsPwm(0.0);
    
    motors.deInit();
    encoders.deInit();
    imu.deInit();
    publisher.deInit();

    return 0;

}