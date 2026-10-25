#include "robot/hal/i_encoder.hpp"
#include "robot/types/result.hpp"
#include "robot/config/encoder_config.hpp"
#include <iostream>

class MockEncoder : public robot::hal::IEncoder
{
    robot::EncoderTick next_;
    robot::EncoderTick last_{};
    bool healthy_{true};
    robot::config::EncoderConfig cfg_;

public:
    ~MockEncoder() = default;
    MockEncoder(robot::config::EncoderConfig cfg) : cfg_(cfg) {
        std::cout << "MockEncoder created with config: gpio_chip=" << cfg_.gpio_chip
                  << ", line_name=" << cfg_.line_name
                  << ", line=" << cfg_.line << std::endl;   
    }
    robot::Result<void> initialize() override { 
        std::cout << "MockEncoder initialized with config: gpio_chip=" << cfg_.gpio_chip
                  << ", line_name=" << cfg_.line_name
                  << ", line=" << cfg_.line << std::endl;
        return {};
     }
    robot::Result<robot::EncoderTick> read() override
    {
        // next_.stamp = std::chrono::system_clock::now();
        next_.ticks++;
        last_ = next_;
        return next_;
    }
    robot::Result<void> run() override
    {

        return {};
    }
    bool is_healthy() const noexcept override { return healthy_; }
};
