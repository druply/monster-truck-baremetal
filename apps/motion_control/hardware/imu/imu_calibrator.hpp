#pragma once

#include <iostream>
#include <fstream>
#include <vector>
#include <array>
#include <optional>
#include <cmath>
#include <nlohmann/json.hpp>

#include "axis_struct.hpp"

using json = nlohmann::json;
using Vector3 = std::array<float, 3>;

// // 1. Core Data Structures
// struct AllAxes {
//     float ax, ay, az;      // accelerometer in m/s^2
//     float gx, gy, gz;      // gyroscope in deg/s
//     float mx, my, mz;      // magnetometer in microtesla (uT)
//     float temp_c;          // temperature in deg Celsius
//     double timestamp;      // timestamp in monotonic seconds
// };

struct CalibrationData {
    Vector3 accel_bias  = {0.0, 0.0, 0.0};
    Vector3 gyro_bias   = {0.0, 0.0, 0.0};
    Vector3 mag_bias    = {0.0, 0.0, 0.0};
    Vector3 mag_scale   = {1.0, 1.0, 1.0};
    Vector3 accel_scale = {1.0, 1.0, 1.0};
    Vector3 gyro_scale  = {1.0, 1.0, 1.0};

    // User-requested format compatibility
    static CalibrationData from_json(const json& j) {
        CalibrationData cal;
        cal.accel_bias  = j.at("accel_bias").get<Vector3>();
        cal.gyro_bias   = j.at("gyro_bias").get<Vector3>();
        cal.mag_bias    = j.at("mag_bias").get<Vector3>();
        cal.mag_scale   = j.at("mag_scale").get<Vector3>();
        cal.accel_scale = j.at("accel_scale").get<Vector3>();
        cal.gyro_scale  = j.at("gyro_scale").get<Vector3>();
        std::cout << "accel_bias" << cal.accel_bias[0] << cal.accel_bias[2] << std::endl;
        return cal;
    }
};

// 2. High-Efficiency Low-Pass Filter
// class LowPassFilter3D {
// private:
//     Vector3 filtered_value = {0.0, 0.0, 0.0};
//     double alpha = 1.0; // Smoothing factor (0.0 to 1.0)
//     bool initialized = false;

// public:
//     explicit LowPassFilter3D(double smoothing_factor) : alpha(smoothing_factor) {}

//     Vector3 update(const Vector3& raw) {
//         if (!initialized) {
//             filtered_value = raw;
//             initialized = true;
//             return filtered_value;
//         }
//         for (size_t i = 0; i < 3; ++i) {
//             filtered_value[i] = alpha * raw[i] + (1.0 - alpha) * filtered_value[i];
//         }
//         return filtered_value;
//     }
// };

class LowPassFilter3D {
private:
    Vector3 filtered_value;
    float alpha;

public:
    // Constexpr constructor ensures zero runtime initialization overhead
    explicit constexpr LowPassFilter3D(float smoothing_factor) noexcept
        : filtered_value{0.0f, 0.0f, 0.0f},
          alpha(smoothing_factor) {}

    // Hard initialization to eliminate branching inside the update loop
    constexpr void reset(const Vector3& initial_raw) noexcept {
        filtered_value = initial_raw;
    }

    // High-efficiency, zero-allocation, branchless update function
    [[nodiscard]] constexpr Vector3 update(const Vector3& raw) noexcept {
        constexpr std::size_t AXES_COUNT = 3;
        
        for (std::size_t i = 0; i < AXES_COUNT; ++i) {
            //filtered_value[i] = (alpha * raw[i]) + ((1.0f - alpha) * filtered_value[i]);
            // Uses 1 subtraction and 1 multiply-accumulate (Fused Multiply-Add)
            filtered_value[i] += alpha * (raw[i] - filtered_value[i]);
        }
        return filtered_value;
    }
};

// 3. Main Processor Class
class IMUProcessor {
private:
    CalibrationData cal;
    LowPassFilter3D accel_filter;
    LowPassFilter3D gyro_filter;
    LowPassFilter3D mag_filter;

public:
    // Uses a standard smoothing factor alpha = 0.15 for high-vibration RPi environments
    IMUProcessor(const CalibrationData& calibration) 
        : cal(calibration), accel_filter(0.1), gyro_filter(0.45), mag_filter(0.1) {  }

    std::optional<AllAxes> process(const std::optional<AllAxes>& raw_data) {
        if (!raw_data.has_value()) {
            return std::nullopt;
        }

        const AllAxes& raw = *raw_data;
        AllAxes processed = raw;

        // Apply standard Accel scale and offset corrections
        Vector3 raw_accel = { static_cast<float>(raw.ax), static_cast<float>(raw.ay), static_cast<float>(raw.az) };
        Vector3 corr_accel;
        for (int i = 0; i < 3; ++i) {
            corr_accel[i] = (raw_accel[i] - cal.accel_bias[i]) * cal.accel_scale[i];
        }
        Vector3 filt_accel = accel_filter.update(corr_accel);

        // Apply standard Gyro scale and offset corrections
        Vector3 raw_gyro = { static_cast<float>(raw.gx), static_cast<float>(raw.gy), static_cast<float>(raw.gz) };
        Vector3 corr_gyro;
        for (int i = 0; i < 3; ++i) {
            corr_gyro[i] = (raw_gyro[i] - cal.gyro_bias[i]) * cal.gyro_scale[i];
        }
        Vector3 filt_gyro = gyro_filter.update(corr_gyro);

        // Apply Hard/Soft Iron Magnetometer corrections
        Vector3 raw_mag = { static_cast<float>(raw.mx), static_cast<float>(raw.my), static_cast<float>(raw.mz) };
        Vector3 corr_mag;
        for (int i = 0; i < 3; ++i) {
            corr_mag[i] = (raw_mag[i] - cal.mag_bias[i]) * cal.mag_scale[i];
        }
        Vector3 filt_mag = mag_filter.update(corr_mag);

        // Repackage data
        processed.ax = static_cast<float>(filt_accel[0]);
        processed.ay = static_cast<float>(filt_accel[1]);
        processed.az = static_cast<float>(filt_accel[2]);

        processed.gx = static_cast<float>(filt_gyro[0]);
        processed.gy = static_cast<float>(filt_gyro[1]);
        processed.gz = static_cast<float>(filt_gyro[2]);

        processed.mx = static_cast<float>(filt_mag[0]);
        processed.my = static_cast<float>(filt_mag[1]);
        processed.mz = static_cast<float>(filt_mag[2]);

        return processed;
    }
};

// 4. File Utility Helper
std::optional<CalibrationData> loadCalibration(const std::string& filename) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Failed to open calibration file: " << filename << "\n";
        return std::nullopt;
    }
    try {
        json j;
        file >> j;
        return CalibrationData::from_json(j);
    } catch (const std::exception& e) {
        std::cerr << "JSON Parsing Error: " << e.what() << "\n";
        return std::nullopt;
    }
}

#if 0
int main() {
    // Load config
    auto cal_opt = loadCalibration("mpu9250_calibration.json");
    if (!cal_opt.has_value()) return 1;

    IMUProcessor processor(*cal_opt);

    // Mock incoming raw sensor stream data
    std::optional<AllAxes> sample_raw = AllAxes{
        .ax = 0.12f, .ay = -0.05f, .az = 9.78f,
        .gx = 0.01f, .gy = -0.02f, .gz = 0.03f,
        .mx = 15.4f, .my = -22.1f, .mz = -5.8f,
        .temp_c = 24.5f,
        .timestamp = 1625091.234
    };

    // Process stream data
    auto clean_data = processor.process(sample_raw);

    if (clean_data.has_value()) {
        std::cout << "--- Calibrated & Filtered Data Output ---\n";
        std::cout << "Timestamp: " << clean_data->timestamp << "\n";
        std::cout << "Accel (m/s^2): [" << clean_data->ax << ", " << clean_data->ay << ", " << clean_data->az << "]\n";
        std::cout << "Gyro (deg/s):  [" << clean_data->gx << ", " << clean_data->gy << ", " << clean_data->gz << "]\n";
        std::cout << "Mag (uT):       [" << clean_data->mx << ", " << clean_data->my << ", " << clean_data->mz << "]\n";
    }

    return 0;
}
#endif