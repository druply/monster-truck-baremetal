// mpu6050_imu.hpp
#pragma once
#include <memory>
#include "robot/hal/i_imu.hpp"
#include "robot/hal/i_clock.hpp"

namespace robot::drivers {
    struct Mpu6050Config {
        std::string i2c_device{"/dev/i2c-1"};
        std::uint8_t address{0x68};
        int sample_rate_hz{100};
    };

    class Mpu6050Imu final : public hal::IImu {
    public:
        Mpu6050Imu(Mpu6050Config cfg, std::shared_ptr<hal::IClock> clock);
        ~Mpu6050Imu() override;

        Result<void> initialize() override;
        Result<ImuSample> read() override;
        [[nodiscard]] bool is_healthy() const noexcept override;

    private:
        struct Impl;                       // pimpl: hides <linux/i2c.h>
        std::unique_ptr<Impl> impl_;
    };
}