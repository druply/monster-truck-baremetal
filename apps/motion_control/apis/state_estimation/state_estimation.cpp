#include "state_estimation.hpp"
#include "hardware/encoders/encoders.hpp"
#include "hardware/imu/imu.hpp"
#include "imu_types.hpp"
#include <cerrno>
#include <iostream>
#include <chrono>
#include <thread>
#include <algorithm> 
#include <numbers>
#include <cmath>

////// Odometry calibration  /////
//constexpr float TIRE_RADIUS			 =           0.0698; // ~0.7 meters
constexpr float TIRE_RADIUS			 =           0.0705; // ~0.7 meters
constexpr float TIRE_DIAMETER        = 			 TIRE_RADIUS*2.0; // meters
constexpr float PULSES_PER_ROTATION  =  		 15;

constexpr float pi_f = std::numbers::pi_v<float>; 

// tire circumfernce = 0.435485 meters
constexpr float DISTANCE_PP = ((pi_f*TIRE_DIAMETER)/PULSES_PER_ROTATION); // distance per tick in meters

// create encoders

 StateEstimation::StateEstimation(IEncoders& encoders, Imu& imu_in)noexcept :  ecdrs(encoders), imu(imu_in)  {

}

StateEstimation::~StateEstimation(){

}   

void StateEstimation::init(void) noexcept {

}

void StateEstimation::run(void) noexcept {
        static int ctr{0};
        static int64_t prev_timestamp_encoders{0};
        int64_t delta_time_encoders_local{0};
        static float delta_time_encoders{0.0f};
        int avg_ctr{0};
        static float prev_encoder_distance{0.0f};
        float encoder_distance{0.0f};
        float encoder_delta_distance{0.0f};
        float encoder_velocity{0.0f};
        static float x_total{0.0f};
        static float y_total{0.0f};


        ctr++;
        
        if (ctr == 3) {
            // get encoder pulses
            int right_ctr = ecdrs.get_right_pulses();
            int left_ctr = ecdrs.get_left_pulses();
            //std::cout << "right: "<< std::to_string(right_ctr) << std::endl;
            //std::cout <<  "left: " << std::to_string(left_ctr) << std::endl;
            // get timestamp
            auto now = std::chrono::system_clock::now();
            // 2. Extract total milliseconds since the clock's epoch using 64-bit integer
            int64_t t = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
            // calculate delta
            delta_time_encoders_local = t - prev_timestamp_encoders;
            // 4. Print directly to cout to utilize the stream formatting options properly
            prev_timestamp_encoders = t;
            // reset counter for 100ms
            ctr = 0;
            // calculate delta in milliseconds
            delta_time_encoders = static_cast<float>(delta_time_encoders_local)/1000.0f; 
            //std:: cout << "delta_time_encoders " << std::to_string(delta_time_encoders) << std::endl;
            //caclaulte avg pulses
            avg_ctr = (right_ctr + left_ctr)/2;
            // calculate encoder total distance
            encoder_distance = static_cast<float>(avg_ctr)*DISTANCE_PP;
            //std:: cout << "encoder_distance " << std::to_string(encoder_distance) << std::endl;
            encoder_delta_distance = encoder_distance - prev_encoder_distance;
            //std:: cout << "encoder_delta_distance " << std::to_string(encoder_delta_distance) << std::endl;
            // calculate encoder velocity
            encoder_velocity = (encoder_delta_distance)/delta_time_encoders; // time step might not be accurate, consider using a timer to read delta time.
            //std:: cout << "encoder_velocity " << std::to_string(encoder_velocity) << std::endl;
            prev_encoder_distance = encoder_distance;
        //} previously calculating encoder here

        // std::cout << "m_state.distance: " << m_state.distance << std::endl;
        // std::cout << "m_state.x: " << m_state.x << std::endl;
        // std::cout << "m_state.y: " << m_state.y << std::endl;
        // std::cout << "m_state.vx: " << m_state.vx << std::endl;
        // std::cout << "m_state.vy: " << m_state.vy << std::endl;
        // std::cout << "m_state.heading: " << m_state.heading << std::endl;
        
     
        // let imu read data
        //imu.read_all_axis();


        ImuData data = imu.getImuData();

        std::cout << "acc: " << data.ax << ", " << data.ay << ", " << data.az << std::endl; 
        std::cout << "gyro: " << data.gx << ", "<< data.gy << ", "<< data.gz << std::endl; 
        std::cout << "heading: " << data.heading << std::endl; 
        //std::cout << "heading offset: " << (heading_angle_offset) << std::endl; 
        //std::cout << "heading total: " << (data.heading - heading_angle_offset) << std::endl; 

        // calculating imu
        float heading_rads = data.heading * M_PI / 180.0;
        float ax_world = data.ax * std::cos(heading_rads) - data.ay*std::sin(heading_rads);
        float ay_world = data.ax * std::sin(heading_rads) + data.ay*std::cos(heading_rads);

        std:: cout << "ax_world " << ax_world << std::endl;
        std:: cout << "ay_world " << ay_world << std::endl;

        static float prev_vx_imu{0.0f};
        static float prev_vy_imu{0.0f};
        float delta_time_s = static_cast<float>(data.delta_time)/1000.0f;
        //std:: cout << "delta_time_s  " << std::to_string(delta_time_s) << std::endl;
        float vx_imu = prev_vx_imu + ax_world*(delta_time_s);
        float vy_imu = prev_vy_imu + ay_world*(delta_time_s);
        

        std:: cout << "vx_imu " << ay_world << std::endl;
        std:: cout << "vy_imu " << vy_imu << std::endl;

        float dx_imu = prev_vx_imu*delta_time_s + (ax_world*(delta_time_s*delta_time_s))/2.0f;
        float dy_imu = prev_vy_imu*delta_time_s + (ay_world*(delta_time_s*delta_time_s))/2.0f;
        std:: cout << "dx_imu " << dx_imu << std::endl;
        std:: cout << "dy_imu " << dy_imu << std::endl;

        prev_vx_imu = vx_imu;
        prev_vy_imu = vy_imu;
        // if car is not moving then dont calculate anything
        if (encoder_velocity == 0.0) {
            dx_imu = 0.0;
            dy_imu = 0.0;
            vx_imu = 0.0;
            vy_imu = 0.0;
        }

        float dx_encoder = encoder_delta_distance*std::cos(heading_rads);
        float dy_encoder = encoder_delta_distance*std::sin(heading_rads);

        std:: cout << "dx_encoder " << dx_encoder << std::endl;
        std:: cout << "dy_encoder " << dy_encoder << std::endl;

        float vx_encoder = encoder_velocity*std::cos(heading_rads);
        float vy_encoder = encoder_velocity*std::sin(heading_rads);

        std:: cout << "vx_encoder " << vx_encoder << std::endl;
        std:: cout << "vy_encoder " << vy_encoder << std::endl;

        const float alpha = 0.8;
        float vx_total = alpha*vx_encoder + (1-alpha)*vx_imu;
        float vy_total = alpha*vy_encoder + (1-alpha)*vy_imu;
        float dx_delta = alpha*dx_encoder + (1-alpha)*dx_imu;
        float dy_delta = alpha*dy_encoder + (1-alpha)*dy_imu;
        std:: cout << "dx_delta " << dx_delta << std::endl;
        std:: cout << "dy_delta " << dy_delta << std::endl;


        static float prev_x_total{0.0f};
        static float prev_y_total{0.0f};
        x_total = prev_x_total + dx_delta;
        y_total = prev_y_total + dy_delta;
        prev_x_total = x_total;
        prev_y_total = y_total;
        float distance_total = std::sqrt((x_total*x_total) + (y_total*y_total));


        m_state.distance = distance_total;
        m_state.x = x_total;
        m_state.y = y_total;
        m_state.vx = vx_total;
        m_state.vy = vy_total;
        m_state.heading = data.heading;


    }// now calculating all data here

        std::this_thread::sleep_for(std::chrono::milliseconds(20));
}

void StateEstimation::deInit(void) noexcept {
    
}