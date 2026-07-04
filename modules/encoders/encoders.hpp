#pragma once

#include "ModuleType.hpp"
#include <thread>
#include <chrono>
#include <atomic>
#include <vector>

class IMessageBroker;

class Encoders : public ModuleType{
	
    // Define the implementation struct inline
    struct Impl {
		 std::atomic<uint64_t> m_right_encoder{0};
        std::atomic_uint32_t m_left_encoder;
        
        // Thread state and control variables
        std::atomic<bool> m_running{false};
        bool m_should_stop = false;
        
        // GPIO resources (if used)
        int right_encoder_gpio{17};
        int left_encoder_gpio{18};

		std::thread worker_thread; 
    };

public:
    Encoders() noexcept;
    ~Encoders();

    void init(void) noexcept override;
    void run(void) noexcept override;
    void deInit(void) noexcept override;
// Non-blocking real-time data accessors using atomic loads
    int64_t get_left_pulses() const noexcept { return m_impl.m_right_encoder.load(std::memory_order_relaxed); }
    int64_t get_right_pulses() const noexcept { return m_impl.m_left_encoder.load(std::memory_order_relaxed); }
private:
    Impl m_impl;  // Now stored on the stack
    
    // Thread entry points
    static void* runner_thread(void*) noexcept;
    void monitor_encoder_right_thread(void) noexcept;
    static void monitor_encoder_left_thread(void*) noexcept;

    // Helper functions for thread management
    bool start_threads();
    void stop_threads();

    // GPIO initialization function (if needed)
    void init_gpio_resources();

	
};