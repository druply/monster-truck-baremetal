#pragma once

#include "ModuleType.hpp"
#include "IEncoders.hpp"
#include <thread>
#include <chrono>
#include <atomic>
#include <vector>

class IMessageBroker;

class Encoders : public ModuleType, public IEncoders {
	
    // Define the implementation struct inline
    struct Impl {
		std::atomic<uint64_t> m_right_encoder{0};
        std::atomic<uint64_t> m_left_encoder{0};

        std::atomic<int64_t> m_left_timestamp_ns{0};
        std::atomic<int64_t> m_right_timestamp_ns{0};
        
        // Thread state and control variables
        std::atomic<bool> m_running{false};
        bool m_should_stop = false;
        
        // GPIO resources (if used)
        int right_encoder_gpio{17};
        int left_encoder_gpio{18};

        std::atomic<bool> right_encoder_fault{false};
        std::atomic<bool> left_encoder_fault{false};

		std::thread right_worker_thread; 
        std::thread left_worker_thread; 
    };

public:
    Encoders() noexcept;
    ~Encoders();

    void init(void) noexcept override;
    void run(void) noexcept override;
    void deInit(void) noexcept override;
// Non-blocking real-time data accessors using atomic loads
    int64_t get_left_pulses() const noexcept override { return m_impl.m_right_encoder.load(std::memory_order_acquire); }
    int64_t get_right_pulses() const noexcept override { return m_impl.m_left_encoder.load(std::memory_order_acquire); }
private:
    Impl m_impl;  // Now stored on the stack
    
    // Thread entry points
    void monitor_encoder_right_thread(void) noexcept;
    void monitor_encoder_left_thread(void) noexcept;
	
};