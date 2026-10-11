// include/robot/drivers/lm393_encoder.hpp
#pragma once
#include <memory>
#include <string>
#include "robot/hal/i_encoder.hpp"

namespace robot::drivers {

struct Lm393EncoderConfig {
    std::string gpio_chip{"/dev/gpiochip0"};
    unsigned int line{17};             // single channel (D0)

    // Holes per revolution of the encoder disk.
    // The sensor produces one pulse per hole, per revolution.
    // Common disks: 20 holes.
    unsigned int holes_per_rev{20};

    // Direction hint provider. Because the LM393 is single-channel,
    // the caller must tell us which way the motor is spinning.
    // We use this to sign the tick counter.
    // false = forward (positive ticks), true = reverse.
    bool invert{false};

    bool use_bias_pull_up{true};
    std::uint32_t debounce_ns{0};
};

class Lm393Encoder final : public hal::IEncoder {
public:
    explicit Lm393Encoder(Lm393EncoderConfig cfg);
    ~Lm393Encoder() override;

    Result<void> initialize() override;
    Result<EncoderTick> read() override;
    [[nodiscard]] bool is_healthy() const noexcept override;

    // Called by the control loop to indicate the current motor direction.
    // Thread-safe (atomic). The encoder uses this to sign future ticks.
    void set_direction(bool reverse) noexcept;

    [[nodiscard]] std::int64_t ticks() const noexcept;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace robot::drivers