
#pragma once
#include "robot/types/result.hpp"
#include "robot/types/pwm_command.hpp"
#include "robot/hal/i_pwm.hpp"
#include "robot/config/pwm_config.hpp"
#include <memory>
#include <string>

namespace robot::hal {
    class PWM : public IPWM {
        robot::config::PWMConfig m_config;
        struct Impl;
        std::unique_ptr<Impl> m_pimpl;
         std::string m_duty_path;

    public:
         PWM(const robot::config::PWMConfig& config);
         ~PWM();
         Result<void> initialize() override;
        Result<void> set(const PWMCommand& cmd) override;
         Result<void> turnOn(void) override;
         Result<void> turnOff(void) override;
    };
} // namespace robot::hal