// include/robot/drivers/pca9685_motor.hpp
#pragma once
#include <memory>
#include <string>
#include "robot/hal/i_motor.hpp"

namespace robot::drivers {

struct Pca9685MotorConfig {
    std::string i2c_device{"/dev/i2c-1"};
    std::uint8_t address{0x40};       // default 0x40
    int pwm_frequency_hz{1000};
    std::uint8_t pwm_channel{0};       // EN pin channel
    std::uint8_t dir_channel{1};       // direction channel (optional)
    bool dir_channel_active{true};
    bool invert{false};                // reverse logic
};

class Pca9685Motor final : public hal::IMotor {
public:
    Pca9685Motor(Pca9685MotorConfig cfg);
    ~Pca9685Motor() override;

    Result<void> initialize() override;
    Result<void> set(const MotorCommand& cmd) override;
    Result<void> stop() override;
    [[nodiscard]] bool is_healthy() const noexcept override;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace robot::drivers