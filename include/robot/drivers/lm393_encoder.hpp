// include/robot/drivers/lm393_encoder.hpp
#pragma once
#include <memory>
#include <string>
#include "robot/hal/i_encoder.hpp"
#include "robot/config/encoder_config.hpp"

namespace robot::drivers {


class Lm393Encoder final : public hal::IEncoder {
public:
    explicit Lm393Encoder(robot::config::EncoderConfig cfg);
    ~Lm393Encoder() override;

    Result<void> initialize() override;
    Result<EncoderTick> read() override;
    Result<void> run() override;
    [[nodiscard]] bool is_healthy() const noexcept override;

    // Called by the control loop to indicate the current motor direction.
    // Thread-safe (atomic). The encoder uses this to sign future ticks.
    void set_direction(bool reverse) noexcept;

    [[nodiscard]] std::int64_t ticks() const noexcept;

private:
    robot::config::EncoderConfig cfg_;
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace robot::drivers