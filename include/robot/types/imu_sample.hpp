// imu_sample.hpp
#pragma once
#include <chrono>
#include <array>

namespace robot {
    using Timestamp = std::chrono::steady_clock::time_point;

    struct Vec3 {
        double x{}, y{}, z{};
    };

    struct ImuSample {
        Timestamp stamp{};
        Vec3 accel{};   // m/s^2
        Vec3 gyro{};    // rad/s
        Vec3 mag{};     // µT (optional)
        double temperature_c{};
    };
}