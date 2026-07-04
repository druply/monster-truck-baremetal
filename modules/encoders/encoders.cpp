#include "encoders.hpp"
#include <iostream>
#include <gpiod.hpp>
#include <expected>
#include <system_error>
#include <thread>

/////// Odometry calibration  /////
const float TIRE_RADIUS			 =           0.0708; // meters
const float TIRE_DIAMETER        = 			 TIRE_RADIUS*2.0; // meters
const float PULSES_PER_ROTATION  =  		 15;

const float DISTANCE_PP = ((3.1416*TIRE_DIAMETER)/PULSES_PER_ROTATION);

// Define exactly one GPIO pin per encoder (BCM numbering)
const int ENC_RIGHT_PIN = 17;
const int ENC_LEFT_PIN = 18;

const std::string chip_name = "gpiochip0"; 


// Type alias for C++23 style error handling
using HardwareResult = std::expected<gpiod::line, std::error_code>;

// Helper function using C++23 std::expected instead of throwing exceptions
HardwareResult setup_gpio_line(unsigned int pin) noexcept {
    // Ensure the compiler knows the pin number is valid for an optimization boost
    [[assume(pin < 32)]]; 
    
    try {
        gpiod::line line = chip.get_line(pin);
        line.request({ "encoder_right", gpiod::line_request::EVENT_RISING_EDGE, 0 });
        return line; // Success case
    } catch (...) {
        // Return a standard error code instead of letting an exception escape
        return std::make_unexpected(std::make_error_code(std::errc::device_or_resource_busy));
    }
}
void Encoders::monitor_encoder_right_thread(void) noexcept {
    
	// Initialize hardware safely using C++23 features
    auto result = setup_gpio_line(m_impl.right_encoder_gpio);
    
    if (!result.has_value()) {
        // Handle hardware initialization failure safely without crashing
        m_impl.right_encoder_fault.store(true, std::memory_order_relaxed);
        return;
    }

    gpiod::line line = std::move(result.value());
    const auto timeout = std::chrono::milliseconds(10);

    // Optimized C++23 Lock-free real-time loop
    while (!stop_token.stop_requested()) {
        if (line.event_wait(timeout)) {
            line.event_read(); 
            
            // Atomically increment with relaxed memory order for peak ARM performance
            m_impl.right_encoder.fetch_add(1, std::memory_order_relaxed);
        }
    }
    
    line.release();
}

void Encoders::init(void) noexcept {
	// 1. Explicitly set the atomic flag using relaxed ordering (no other threads exist yet)
    m_impl.m_running.store(true, std::memory_order_relaxed);

    // 2. Spawn the thread only after the flag is guaranteed to be true
    m_impl.worker_thread = std::thread(&Encoders::monitor_encoder_right_thread, this, nullptr);
}

void Encoders::run(void) noexcept {
        
}

void Encoders::deInit(void) noexcept {
    // Explicitly requesting a stop signals the stop_token and joins the thread.
    // This blocks for a maximum of 10ms (our line.event_wait timeout) and safely exits.
   // m_impl.worker_thread.request_stop();
}