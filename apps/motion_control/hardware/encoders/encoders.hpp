#pragma once

#include "ModuleType.hpp"
#include "IEncoders.hpp"

class Encoders : public ModuleType, public IEncoders {
	
    struct Impl;
    std::unique_ptr<Impl> m_impl;  // Pimpl idiom for encapsulation and reduced compile-time dependencies

public:
    Encoders() noexcept;
    ~Encoders();

    void init(void) noexcept override;
    void run(void) noexcept override;
    void deInit(void) noexcept override;
// Non-blocking real-time data accessors using atomic loads
    int64_t get_left_pulses() const noexcept override;
    int64_t get_right_pulses() const noexcept override ;
private:
    // Thread entry points
    void monitor_encoder_right_thread(void) noexcept;
    void monitor_encoder_left_thread(void) noexcept;	
};