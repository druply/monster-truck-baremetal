#pragma once
#include <array>

using Vector3 = std::array<float, 3>;

struct AllAxes {
    // accelerometer in m/s^2
    float ax, ay, az;
    // gyroscope in deg/s
    float gx, gy, gz;
    // magnetometer in microtesla (uT)
    float mx, my, mz;
    // temperature in deg Celsius
    float temp_c;
    // timestamp in monotonic seconds (double)
    uint64_t timestamp;
};


struct ImuData {
    // accelerometer in m/s^2
    float ax, ay, az;
    // gyroscope in deg/s
    float gx, gy, gz;
    // magnetometer in microtesla (uT)
    float mx, my, mz; 
    // angle data
    float heading, pitch, roll;

    uint64_t delta_time;
};