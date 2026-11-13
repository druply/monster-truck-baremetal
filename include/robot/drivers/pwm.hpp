
#pragma once
#include "robot/hal/i_pwm.hpp"
#include "robot/config/pwm_config.hpp"
#include <memory>

namespace robot::drivers {
    class Pca9685Pwm final : public hal::IPWM {
        struct Impl;
        std::unique_ptr<Impl> m_impl;

    public:
        explicit Pca9685Pwm(config::PWMConfig config);
        ~Pca9685Pwm() override;
        Pca9685Pwm(const Pca9685Pwm&) = delete;
        Pca9685Pwm& operator=(const Pca9685Pwm&) = delete;

        Result<void> initialize() override;
        Result<void> set(const PWMCommand& cmd) override;
        Result<void> turnOn() override;
        Result<void> turnOff() override;
    };
} // namespace robot::drivers