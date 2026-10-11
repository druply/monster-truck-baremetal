// include/robot/drivers/mpu9250_imu.hpp
#pragma once
#include <memory>
#include <string>
#include "robot/hal/i_imu.hpp"
#include "robot/hal/i_clock.hpp"

namespace robot::drivers {

struct Mpu9250Config {
    std::string i2c_device{"/dev/i2c-1"};
    std::uint8_t mpu_address{0x68};      // AD0=0 -> 0x68, AD0=1 -> 0x69
    bool enable_magnetometer{true};
    int sample_rate_hz{100};
    int accel_range_g{8};                 // ±8 g
    int gyro_range_dps{1000};             // ±1000 °/s
};

class Mpu9250Imu final : public hal::IImu {
public:
    Mpu9250Imu(Mpu9250Config cfg, std::shared_ptr<hal::IClock> clock);
    ~Mpu9250Imu() override;

    Result<void> initialize() override;
    Result<ImuSample> read() override;
    [[nodiscard]] bool is_healthy() const noexcept override;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace robot::drivers