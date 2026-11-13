// imu_sample.hpp
#pragma once
#include "timestamp.hpp"
#include <cstdint>

namespace robot
{

    struct Vec3
    {
        double x{}, y{}, z{};
    };

    struct ImuSample
    {
        std::uint64_t timestamp{};
        Vec3 accel{}; // m/s^2
        Vec3 gyro{};  // rad/s
        Vec3 mag{};   // µT (optional)
        double temperature_c{};
    };
}