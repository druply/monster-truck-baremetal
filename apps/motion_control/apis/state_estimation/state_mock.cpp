#include "state_estimation.hpp"
#include "modules/encoders/encoders.hpp"
#include "modules/imu/imu.hpp"
#include "modules/imu/axis_struct.hpp"
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

// x,y,vx,vy,head, dis
std::vector<State_t> states ={
        {0.0f,  0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
        {0.1f,  0.0f, 0.5f, 0.0f, 0.0f, 0.0f},
        {0.2f,  0.0f, 0.5f, 0.0f, 0.0f, 0.0f},
        {0.3f,  0.0f, 0.5f, 0.0f, 0.0f, 0.0f},
        {0.4f,  0.0f, 0.5f, 0.0f, 0.0f, 0.0f},
        {0.5f,  0.0f, 0.5f, 0.0f, 0.0f, 0.0f},
        {0.6f,  0.0f, 0.5f, 0.0f, 0.0f, 0.0f},
        {0.7f,  0.0f, 0.5f, 0.0f, 0.0f, 0.0f},
        {0.8f,  0.0f, 0.5f, 0.0f, 0.0f, 0.0f},
        {0.9f,  0.0f, 0.5f, 0.0f, 0.0f, 0.0f},
        {1.0f,  0.0f, 0.5f, 0.0f, 0.0f, 0.0f},
        {1.1f,  0.0f, 0.5f, 0.0f, 0.0f, 0.0f},
        {1.2f,  0.0f, 0.5f, 0.0f, 0.0f, 0.0f},
        {1.3f,  0.0f, 0.5f, 0.0f, 0.0f, 0.0f},
        {1.4f,  0.0f, 0.5f, 0.0f, 0.0f, 0.0f},
        {1.5f,  0.01f, 0.5f, 0.0f, 0.0f, 0.0f},
        {1.6f,  0.02f, 0.5f, 0.0f, 0.0f, 0.0f},
        {1.7f,  0.03f, 0.5f, 0.0f, 0.0f, 0.0f},
        {1.8f,  0.04f, 0.5f, 0.0f, 0.0f, 0.0f},
        {1.9f,  0.1f, 0.5f, 0.0f, 0.0f, 0.0f},
        {2.0f,  0.2f, 0.5f, 0.0f, 0.0f, 0.0f},
        {2.1f,  0.3f, 0.5f, 0.0f, 0.0f, 0.0f},
        {2.2f,  0.5f, 0.5f, 0.0f, 0.0f, 0.0f},
        {2.3f,  0.7f, 0.5f, 0.0f, 0.0f, 0.0f},
        {2.4f,  0.8f, 0.5f, 0.0f, 0.0f, 0.0f},
        {2.5f,  0.9f, 0.5f, 0.0f, 0.0f, 0.0f},
    };

 StateEstimation::StateEstimation() noexcept {

}

StateEstimation::~StateEstimation(){

}   

void StateEstimation::init(void) noexcept {

}

void StateEstimation::run(void) noexcept {

    static int ctr{0};
    if (ctr<states.size()) {


    m_state.distance = states.at(ctr).distance;
    m_state.x = states.at(ctr).x;
    m_state.y = states.at(ctr).y;
    m_state.vx = states.at(ctr).vx;
    m_state.vy = states.at(ctr).vy;
    m_state.heading = states.at(ctr).heading;
        ctr++;
        }



    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
}

void StateEstimation::deInit(void) noexcept {

}