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


 // Define the implementation struct inline
    struct Encoders::Impl {
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

        Impl() = default;
        ~Impl() {
            if (right_worker_thread.joinable()) {
                right_worker_thread.join();
            }
            if (left_worker_thread.joinable()) {
                left_worker_thread.join();
            }
        }
    };


    Encoders::Encoders() noexcept: m_impl(std::make_unique<Impl>()) {}
    Encoders::~Encoders(){}


void Encoders::monitor_encoder_right_thread(void) noexcept {

    using namespace std::chrono_literals;
    using namespace std::chrono;

    while(m_impl->m_running.load(std::memory_order_relaxed)) {
        
        std::this_thread::sleep_for(std::chrono::milliseconds(50));

        m_impl->m_right_encoder.fetch_add(1, std::memory_order_release);
        int64_t ts = duration_cast<nanoseconds>(steady_clock::now().time_since_epoch()).count();
        m_impl->m_right_timestamp_ns.store(ts, std::memory_order_relaxed);
    }

        std::cout << "exiting monitor_encoder_right_thread" << std::endl;

}

    int64_t Encoders::get_left_pulses() const noexcept  { return m_impl->m_left_encoder.load(std::memory_order_acquire); }
    int64_t Encoders::get_right_pulses() const noexcept  { return m_impl->m_right_encoder.load(std::memory_order_acquire); }


void Encoders::monitor_encoder_left_thread(void) noexcept {
    using namespace std::chrono_literals;
    using namespace std::chrono;

    while(m_impl->m_running.load(std::memory_order_relaxed)) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));

        m_impl->m_left_encoder.fetch_add(1, std::memory_order_release);
        int64_t ts = duration_cast<nanoseconds>(steady_clock::now().time_since_epoch()).count();
        m_impl->m_left_timestamp_ns.store(ts, std::memory_order_relaxed);
    }

        std::cout << "exiting monitor_encoder_left_thread" << std::endl;

}
void Encoders::init(void) noexcept {
    std::cout << "[Encoders] init" << std::endl;
    // 1. Explicitly set the atomic flag using relaxed ordering (no other threads exist yet)
    m_impl->m_running.store(true, std::memory_order_release);

    // 2. Spawn the thread only after the flag is guaranteed to be true
    m_impl->right_worker_thread = std::thread(&Encoders::monitor_encoder_right_thread, this);
    m_impl->left_worker_thread = std::thread(&Encoders::monitor_encoder_left_thread, this);
}

void Encoders::run(void) noexcept {
    std::cout << "[Encoders] run" << std::endl;
    using namespace std::chrono_literals;
    std::this_thread::sleep_for(1s); // sleep 2 seconds

}

void Encoders::deInit(void) noexcept {
    std::cout << "[Encoders] deInit" << std::endl;
    // Explicitly requesting a stop signals the stop_token and joins the thread.
    // This blocks for a maximum of 10ms (our line.event_wait timeout) and safely exits.
    //m_impl.worker_thread.request_stop();
     m_impl->m_running.store(false, std::memory_order_release);
     // join thread so we wait for it to finish
    //  m_impl->right_worker_thread.join();
    //  m_impl->left_worker_thread.join();
}