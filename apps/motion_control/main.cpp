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

std::array<ModuleType*, 4> modules = {&motors, &encoders, &imu, &publisher};

int main() {
    // Initialize modules
    for (auto module : modules) {
        module->init();
    }
    
    motors.setMotorsDirections(MotorsDirection_t::FORWARD);
    motors.setMotorsPwm(1.0);
    motors.setSteeringAngle(0.0);

    // Run modules
    for (auto module : modules) {
        module->run();
        std::cout << "encoder right: " << encoders.get_right_pulses() << std::endl;
    }


    // deinit modules
    motors.setMotorsPwm(0.0);
    for (auto module : modules) {
        module->deInit();
    }
    return 0;

}