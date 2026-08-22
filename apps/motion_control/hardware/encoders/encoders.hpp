#pragma once

#include <thread>

class Encoders {
	
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

public:
    Encoders() noexcept;
    ~Encoders();

    void init(void) noexcept;
    void run(void) noexcept;
    void deInit(void) noexcept;
// Non-blocking real-time data accessors using atomic loads
    int64_t get_left_pulses() const noexcept;
    int64_t get_right_pulses() const noexcept ;
private:
    // Thread entry points
    void monitor_encoder_right_thread(void) noexcept;
    void monitor_encoder_left_thread(void) noexcept;	
};