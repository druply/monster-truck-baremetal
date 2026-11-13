#pragma once
#include <string>

namespace robot::config
{

    struct PWMConfig
    {
        std::string duty_path{"/sys/class/pwm/pwmchip0/pwm"};
        int channel{};
        unsigned int max_period{0};

    };

} // namespace robot::config