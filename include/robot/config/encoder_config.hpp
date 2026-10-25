#pragma once
#include <string>



// struct Lm393EncoderConfig {
//     std::string gpio_chip{"gpiochip0"};
//     std::string line_name{"encoder_right"};
//     unsigned int line{17};             // single channel (D0)

//     // Holes per revolution of the encoder disk.
//     // The sensor produces one pulse per hole, per revolution.
//     // Common disks: 20 holes.
//     //unsigned int holes_per_rev{20};

//     // Direction hint provider. Because the LM393 is single-channel,
//     // the caller must tell us which way the motor is spinning.
//     // We use this to sign the tick counter.
//     // false = forward (positive ticks), true = reverse.
//     bool invert{false};

//     // bool use_bias_pull_up{true};
//     // std::uint32_t debounce_ns{0};
// };


namespace robot::config
{

    struct EncoderConfig
    {
        std::string gpio_chip{"gpiochip0"};
        std::string line_name{};
        unsigned int line{17};             // single channel (D0)

    };

} // namespace robot::app