#include "encoders.hpp"
#include <iostream>
#include <thread>
#include <chrono>

/////// Odometry calibration  /////
const float TIRE_RADIUS			 =           0.0708; // meters
const float TIRE_DIAMETER        = 			 TIRE_RADIUS*2.0; // meters
const float PULSES_PER_ROTATION  =  		 15;

const float DISTANCE_PP = ((3.1416*TIRE_DIAMETER)/PULSES_PER_ROTATION);

// Define exactly one GPIO pin per encoder (BCM numbering)
const int ENC_RIGHT_PIN = 17;
const int ENC_LEFT_PIN = 18;

const std::string chip_name = "gpiochip0"; 


    Encoders::Encoders() noexcept{}
    Encoders::~Encoders(){}

void Encoders::monitor_encoder_right_thread(void) noexcept {

    using namespace std::chrono_literals;
    std::this_thread::sleep_for(1s); // sleep 2 seconds

}

void Encoders::init(void) noexcept {
    // 1. Explicitly set the atomic flag using relaxed ordering (no other threads exist yet)
    m_impl.m_running.store(true, std::memory_order_relaxed);

    // 2. Spawn the thread only after the flag is guaranteed to be true
    m_impl.worker_thread = std::thread(&Encoders::monitor_encoder_right_thread, this);
}

void Encoders::run(void) noexcept {
        
}

void Encoders::deInit(void) noexcept {
    // Explicitly requesting a stop signals the stop_token and joins the thread.
    // This blocks for a maximum of 10ms (our line.event_wait timeout) and safely exits.
    //m_impl.worker_thread.request_stop();
     m_impl.m_running.store(false, std::memory_order_relaxed);
}