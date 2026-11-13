#pragma once
#include <cstdint>
#include <string>

namespace robot::config
{

    struct PWMConfig
    {
        std::string i2c_device{"/dev/i2c-1"};
        std::uint8_t address{0x40};
        int frequency_hz{1000};
        std::uint8_t channel{};
    };

} // namespace robot::config