#include <boost/asio.hpp>
#include <thread>
#include <vector>
#include <iostream>

#include "modules/encoders/encoders.hpp"
#include "modules/motors/motors.hpp"
#include "modules/imu/mpu9250.hpp"
#include "apis/state_estimation/state_estimation.hpp"
#include "apis/trajectory_follower/trajectory_follower.hpp"
#include "apis/trajectory_follower/trajectory_type.hpp"
#include <iomanip>
#include <numbers>
#include <algorithm>

//#include "modules/imu/imu_calibrator.hpp"
#include "modules/imu/axis_struct.hpp"

#include <chrono>

#if 0
int testRun(void)
{
    using namespace std::chrono_literals;
    using namespace std::chrono;

    // Encoders ecdrs;
    // Motors motors;

    // mpu0250 config
    mpu9250::Config cfg;
    cfg.i2c_bus = 1;
    cfg.mpu_addr = 0x68;
    cfg.accel_fsr_g = 16;
    cfg.gyro_fsr_dps = 2000;
    cfg.smplrt_div = 4;
    cfg.dlpf_cfg = 3;
    // create mpu9250 instance
    mpu9250::MPU9250 dev(cfg);

    // Load config
    auto cal_opt = loadCalibration("mpu9250_calibration.json");
    if (!cal_opt.has_value())
        return 1;

    std::cout << "loaded calib data" << cal_opt->accel_bias[0] << cal_opt->accel_bias[1] << cal_opt->accel_bias[2] << std::endl;

    IMUProcessor calib(*cal_opt);

    // ecdrs.init();
    // motors.init();

    // initialize mpu9250
    auto ec = dev.initialize();
    if (ec)
    {
        std::cerr << "MPU9250 init failed: " << ec.message() << " (code " << ec.value() << ")\n";
        return 1;
    }

    // motors.setMotorsDirections(MotorsDirection_t::FORWARD);
    // motors.setMotorsPwm(1.0);
    // motors.setSteeringAngle(40.0);

    for (int i = 0; i < 200; i++)
    {

        // std::this_thread::sleep_for(1s); // sleep 2 seconds
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        // ecdrs.run();
        // std::cout << "right: "<< std::to_string(ecdrs.get_left_pulses()) << std::endl;
        // std::cout <<  "left: " << std::to_string(ecdrs.get_right_pulses()) << std::endl;
        //  if (ecdrs.get_left_pulses() > 80) {
        //      break;
        //  }
        //  if (ecdrs.get_right_pulses() > 80) {
        //       break;
        //  }
        //  read mpu9250 data
        auto data = dev.read_all();
        if (!data)
        {
            std::cerr << "read failed\n";
        }
        else
        {
            auto &d = *data;
            std::cout << std::fixed << std::setprecision(3)
                      << "t=" << d.timestamp
                      << " Acc(m/s2)=[" << d.ax << "," << d.ay << "," << d.az << "]"
                      << " Gyro(dps)=[" << d.gx << "," << d.gy << "," << d.gz << "]"
                      << " Mag(uT)=[" << d.mx << "," << d.my << "," << d.mz << "]"
                      << " Temp(C)=" << d.temp_c << "\n";
        }

        const auto &clean_data = calib.process(data); // thread-safe call
        std::cout << "calibrated data:" << std::endl;

        if (clean_data.has_value())
        {
            std::cout << "--- Calibrated & Filtered Data Output ---\n";
            std::cout << "Timestamp: " << clean_data->timestamp << "\n";
            std::cout << "Accel (m/s^2): [" << clean_data->ax << ", " << clean_data->ay << ", " << clean_data->az << "]\n";
            std::cout << "Gyro (deg/s):  [" << clean_data->gx << ", " << clean_data->gy << ", " << clean_data->gz << "]\n";
            std::cout << "Mag (uT):       [" << clean_data->mx << ", " << clean_data->my << ", " << clean_data->mz << "]\n";
        }
    }

    // motors.setMotorsPwm(0.0);
    // motors.setSteeringAngle(-40.0);
    // std::this_thread::sleep_for(5s); // sleep 2 seconds
    // motors.setSteeringAngle(0.0);
    // ecdrs.deInit();
    // motors.deInit();

    dev.close(); // deinit mpu9250

    return 0;
}

#endif

#if 0
static void displayImu(std::optional<AllAxes> data){
    AllAxes &d = *data;
    std::cout << std::fixed << std::setprecision(3)
                      << "t=" << d.timestamp
                      << " Acc(m/s2)=[" << d.ax << "," << d.ay << "," << d.az << "]"
                      << " Gyro(dps)=[" << d.gx << "," << d.gy << "," << d.gz << "]"
                      << " Mag(uT)=[" << d.mx << "," << d.my << "," << d.mz << "]"
                      << " Temp(C)=" << d.temp_c << "\n";
}

//Global static declarations for all instances

// mpu0250 config
static mpu9250::Config cfg{};
    
// create mpu9250 instance
static mpu9250::MPU9250 dev(cfg);

// create encoders
static Encoders ecdrs;

static Motors motors;

// create odometry and pass reference for encoders
static Odometry odometry(ecdrs, dev);


int testIntegration() {
using namespace std::chrono_literals;
    using namespace std::chrono;
    int right_ctr;
    int left_ctr;

    // initialize mpu9250
    auto ec = dev.initialize();

    motors.init(); 

    ecdrs.init();

    motors.setMotorsDirections(MotorsDirection_t::FORWARD);
    motors.setMotorsPwm(0.6);
    motors.setSteeringAngle(-30.0);

    for (int i =0; i<10000;i++) {
    auto data = dev.read_all();
    //odometry.run();
       if (!data)
        {
            std::cerr << "read failed\n";
        }
        else
        {
            //auto &d = *data;
            displayImu(data);
            
        }
        right_ctr = ecdrs.get_right_pulses();
        left_ctr = ecdrs.get_left_pulses();
        std::cout << "right: "<< std::to_string(right_ctr) << std::endl;
        std::cout <<  "left: " << std::to_string(left_ctr) << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        if (left_ctr > 90) {
            break;
        }
    }

    motors.deInit();
    motors.setSteeringAngle(0.0);

    dev.close(); // deinit mpu9250

    ecdrs.deInit();

    return 0;

}



int main()
{
    //return testRun();
    //return testIntegration();


    // initialize mpu9250
    auto ec = dev.initialize();

    motors.init(); 

    ecdrs.init();

    odometry.init();

    motors.setMotorsDirections(MotorsDirection_t::FORWARD);
    motors.setMotorsPwm(0.6);
    motors.setSteeringAngle(0.0);

    int ctr{0};

    for (int i =0; i<300;i++) {
        using namespace std::chrono_literals;
        using namespace std::chrono;
     
        odometry.run();

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    motors.setMotorsPwm(0.0); // stop car

    // run a few cycles to measure distance
    for (int i =0; i<200;i++) {
        using namespace std::chrono_literals;
        using namespace std::chrono;
     
        odometry.run();

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    motors.deInit();

    ecdrs.deInit();

    dev.close();

    odometry.deInit();
}

#endif


static StateEstimation state;
static Motors motors;
static TrajectoryFollower traj_follower;

constexpr float pi_f = std::numbers::pi_v<float>; 

static void setSteering(float steering){
    float local_steer{0.0};
    local_steer = (steering*180.0)/pi_f;
    std::cout << "steering angle: " << local_steer << std::endl;
    //motors.setSteeringAngle(local_steer);
    motors.setSteeringAngle(0.0);
}

static void setVelocity(float velocity){
    motors.setMotorsPwm(1.0);
}

int main() {

    motors.init(); 
    state.init();
    traj_follower.init();

    motors.setMotorsDirections(MotorsDirection_t::FORWARD);
    motors.setMotorsPwm(1.0);
    motors.setSteeringAngle(0.0);


    // Initialize the vector with the CSV data
    // short test
    
    std::vector<TrajectoryPoint> trajectory = {
        {0.0f,  0.0f,  1.0f},
        {0.2f,  0.0f,  1.0f},
        {0.4f,  0.0f,  1.0f},
        {0.6f,  0.0f,  1.0f},
        {0.8f,  0.0f,  1.0f},
        {1.0f,  0.0f,  1.0f},
        {1.2f,  0.0f,  1.0f},
        {1.4f,  0.1f,  1.0f},
        {1.6f,  0.2f,  1.0f},
        {1.8f,  0.3f,  1.0f},
        {2.0f,  0.4f,  1.0f},
        {2.2f,  0.5f,  1.0f},
        {2.4f,  0.6f,  1.0f},
        {2.6f,  0.7f,  1.0f},
        {2.8f,  0.7f,  1.0f},
        // {3.0f,  0.7f,  1.0f},
        // {3.2f,  0.7f,  1.0f},
        // {3.4f,  0.7f,  1.0f},
        // {3.6f,  0.7f,  1.0f},
    };
    
    /*

     std::vector<TrajectoryPoint> trajectory = {
        {0.0f, 0.0f, 0.0f},
        {0.2f, 0.0f, 0.5f},
        {0.4f, 0.0f, 0.5f},
        {0.6f, 0.0f, 0.5f},
        {0.8f, 0.0f, 1.0f},
        {1.0f, 0.0f, 1.0f},
        {1.2f, 0.0f, 1.0f},
        {1.4f, 0.0f, 1.0f},
        {1.6f, 0.0f, 1.0f},
        {1.8f, 0.0f, 1.0f},
        {2.0f, 0.0f, 1.0f},
        {2.2f, 0.0f, 1.0f},
        {2.4f, 0.0f, 1.0f},
        {2.6f, 0.0f, 1.0f},
        {2.8f, 0.0f, 1.0f},
        {3.0f, 0.0f, 1.0f},
        {3.2f, 0.0f, 1.0f},
        {3.4f, 0.0f, 1.0f},
        {3.6f, 0.0f, 1.0f},
        {3.8f, 0.0f, 1.0f},
        {4.0f, 0.0f, 1.0f},
        {4.2f, 0.0f, 1.0f},
        {4.4f, 0.0f, 1.0f},
        {4.6f, 0.0f, 1.0f},
        {4.8f, 0.0f, 1.0f},
        {5.0f, 0.0f, 1.0f},
        {5.13f, 0.01f, 0.8f},
        {5.25f, 0.02f, 0.8f},
        {5.38f, 0.05f, 0.8f},
        {5.50f, 0.09f, 0.8f},
        {5.62f, 0.13f, 0.8f},
        {5.73f, 0.19f, 0.8f},
        {5.84f, 0.26f, 0.8f},
        {5.94f, 0.33f, 0.8f},
        {6.04f, 0.42f, 0.8f},
        {6.50f, 0.68f, 0.8f},
        {6.61f, 0.81f, 0.8f},
        {6.70f, 0.95f, 0.8f},
        {6.83f, 0.95f, 0.8f},
        {6.96f, 0.95f, 0.8f},
        {7.08f, 0.95f, 1.0f},
        {7.28f, 0.95f, 1.0f},
        {7.48f, 0.95f, 1.0f},
        {7.68f, 0.95f, 1.0f},
        {7.88f, 0.95f, 1.0f},
        {8.08f, 0.95f, 1.0f},
        {8.28f, 0.95f, 1.0f},
        {8.48f, 0.95f, 1.0f},
        {8.68f, 0.95f, 1.0f},
        {8.88f, 0.95f, 1.0f},
        {9.08f, 0.95f, 0.5f},
        {9.28f, 0.95f, 0.0f},
        {9.48f, 0.95f, 0.0f}
    };
*/

   // let filters stabilize
    for(int i=0; i< 500; i++) {
        state.run();
    }

    traj_follower.setTrajectory(trajectory);

    while(true) {

        state.run();
        State_t m_state = state.getState();
        CarState car_state;
        car_state.x = m_state.x;
        car_state.y = m_state.y;
        car_state.theta = (m_state.heading*pi_f)/180.0f; // convert deg to rads

        traj_follower.setCarState(car_state);
        traj_follower.run();
        auto [steering, velocity] = traj_follower.getControlVariables();
        //setSteering(steering);
        //setVelocity(velocity);

        std::cout << "clamped steering: " << steering << std::endl;
        std::cout << "target velocity: " << velocity << std::endl;
        std::cout << "x: " << m_state.x << std::endl;
        std::cout << "y: " << m_state.y << std::endl;
        std::cout << "vx: " << m_state.vx << std::endl;
        std::cout << "vy: " << m_state.vy << std::endl;
        std::cout << "heading: " << m_state.heading << std::endl;
        std::cout << "distance: " << m_state.distance << std::endl;

        std::cout << "Processing data..." << std::flush;

        // if (m_state.x > 1.0) {
        //     motors.setSteeringAngle(-20.0);
        // }

        if (m_state.x > 2.7) {
            break;
        }


    }

        motors.setMotorsPwm(0.0);
        motors.deInit();
    return 0;

}