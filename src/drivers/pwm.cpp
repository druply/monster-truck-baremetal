
#include "robot/drivers/pwm.hpp"

#include <fstream>

namespace robot::hal
{

    struct PWM::Impl
    {
        // Implementation details go here
    };

    PWM::PWM(const robot::config::PWMConfig &config) : m_config(config), m_pimpl(std::make_unique<Impl>()) {}

    PWM::~PWM() = default;

    Result<void> PWM::initialize()
    {
        // Implementation goes here
        m_duty_path = m_config.duty_path + std::to_string(m_config.channel) + "/duty_cycle";
        return Result<void>{};
    }

    Result<void> PWM::set(const PWMCommand &cmd)
    {
        double duty = cmd.duty;
        // Implementation goes here
        if (duty < 0.0)
            duty = 0.0;
        if (duty > 1.0)
            duty = 1.0;

        // std::cout << "duty: " << std::to_string(duty) << std::endl;
        // std::cout << "max_period: " << std::to_string(m_config.max_period) << std::endl;

        int duty_ns = static_cast<int>(duty * static_cast<double>(m_config.max_period));
        // std::cout << "writing period: " << duty_ns << std::endl;
        std::ofstream file(m_duty_path);
        if (file.is_open())
        {
            file << duty_ns;
            file.close();
        }
        else
        {
            //std::cerr << "Error: Could not open " << m_duty_path << ". Check permissions!" << std::endl;
            return std::unexpected(Error{Module::Motor, ErrorCode::IoFailure, "Failed to open duty cycle file"});
        }
        return {};
    }

    Result<void> PWM::turnOn(void)
    {
        set(PWMCommand{.duty = 1.0});
        return {};
    }

    Result<void> PWM::turnOff(void)
    {
        set(PWMCommand{.duty = 0.0});
        return {};
    }

} // namespace robot::hal