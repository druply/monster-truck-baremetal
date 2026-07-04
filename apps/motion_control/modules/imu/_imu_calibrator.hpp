// imu_calibrator.hpp
// C++20 header-only IMU calibrator/filter for MPU9250
// Requires nlohmann/json.hpp
// Usage:
//   #include "imu_calibrator.hpp"
//   ImuCalibrator calib("calib.json", 0.3);           // load file, lpf alpha
//   AllAxes raw = ...;                                // from driver
//   auto out = calib.process(raw);                    // thread-safe

#pragma once
#include <array>
#include <atomic>
#include <fstream>
#include <mutex>
#include <shared_mutex>
#include <string>
#include <stdexcept>
#include <sstream>
#include <vector>
#include "nlohmann/json.hpp"
using json = nlohmann::json;

#include "axis_struct.hpp"

struct Calibrated {
    float ax, ay, az;
    float gx, gy, gz;
    float mx, my, mz;
    float temp_c;
    double timestamp;
};

class ImuCalibrator {
public:
    // Load calibration and set LPF alpha (0..1). alpha==1 -> passthrough (no filtering).
    explicit ImuCalibrator(const std::string &jsonPath, double lpfAlpha = 0.5)
        : lpfAlpha_(clampAlpha(lpfAlpha))
    {
        loadCalibration(jsonPath);
        resetFilters();
    }

    // Process one raw sample -> calibrated+filtered output. Thread-safe.
    Calibrated process(const std::optional<AllAxes> &in) {
        // copy calibration under shared lock for low-latency concurrent access
        std::shared_lock lock(calibMutex_);
        // convert accel units: calibration accel_bias assumed in g (same as prior tool)
        constexpr double G = 9.80665;
        // remove accel bias (in g), apply accel scale
        double ax = (in.ax / G) - calib_.accel_bias[0];
        double ay = (in.ay / G) - calib_.accel_bias[1];
        double az = (in.az / G) - calib_.accel_bias[2];
        ax *= calib_.accel_scale; ay *= calib_.accel_scale; az *= calib_.accel_scale;
        // gyro: remove bias (deg/s) and apply scale
        double gx = (in.gx) - calib_.gyro_bias[0];
        double gy = (in.gy) - calib_.gyro_bias[1];
        double gz = (in.gz) - calib_.gyro_bias[2];
        gx *= calib_.gyro_scale; gy *= calib_.gyro_scale; gz *= calib_.gyro_scale;
        // mag: remove hard-iron bias then apply soft-iron 3x3 matrix
        double mx = in.mx - calib_.mag_bias[0];
        double my = in.my - calib_.mag_bias[1];
        double mz = in.mz - calib_.mag_bias[2];
        double mx_c = calib_.mag_scale[0][0]*mx + calib_.mag_scale[0][1]*my + calib_.mag_scale[0][2]*mz;
        double my_c = calib_.mag_scale[1][0]*mx + calib_.mag_scale[1][1]*my + calib_.mag_scale[1][2]*mz;
        double mz_c = calib_.mag_scale[2][0]*mx + calib_.mag_scale[2][1]*my + calib_.mag_scale[2][2]*mz;
        lock.unlock();

        // Filter (per-channel IIR). Use small critical section to protect filter state.
        std::scoped_lock flock(filterMutex_);
        double ax_f = filtAccel_[0].filter(ax, lpfAlpha_);
        double ay_f = filtAccel_[1].filter(ay, lpfAlpha_);
        double az_f = filtAccel_[2].filter(az, lpfAlpha_);
        double gx_f = filtGyro_[0].filter(gx, lpfAlpha_);
        double gy_f = filtGyro_[1].filter(gy, lpfAlpha_);
        double gz_f = filtGyro_[2].filter(gz, lpfAlpha_);
        double mx_f = filtMag_[0].filter(mx_c, lpfAlpha_);
        double my_f = filtMag_[1].filter(my_c, lpfAlpha_);
        double mz_f = filtMag_[2].filter(mz_c, lpfAlpha_);

        Calibrated out;
        out.ax = static_cast<float>(ax_f * G);
        out.ay = static_cast<float>(ay_f * G);
        out.az = static_cast<float>(az_f * G);
        out.gx = static_cast<float>(gx_f);
        out.gy = static_cast<float>(gy_f);
        out.gz = static_cast<float>(gz_f);
        out.mx = static_cast<float>(mx_f);
        out.my = static_cast<float>(my_f);
        out.mz = static_cast<float>(mz_f);
        out.temp_c = in.temp_c;
        out.timestamp = in.timestamp;
        return out;
    }

    // Reload calibration file at runtime. Thread-safe.
    void reloadCalibration(const std::string &jsonPath) {
        auto newCal = loadCalibrationFromFile(jsonPath);
        {
            std::unique_lock lock(calibMutex_);
            calib_ = std::move(newCal);
        }
        resetFilters();
    }

    // Set LPF alpha at runtime (0..1). Thread-safe.
    void setLpfAlpha(double a) {
        lpfAlpha_.store(clampAlpha(a), std::memory_order_relaxed);
    }

    double lpfAlpha() const noexcept { return lpfAlpha_.load(std::memory_order_relaxed); }

private:
    // Internal calibration representation
    struct Calibration {
        std::array<double,3> accel_bias{};
        std::array<double,3> gyro_bias{};
        std::array<double,3> mag_bias{};
        std::array<std::array<double,3>,3> mag_scale{};
        double accel_scale = 1.0;
        double gyro_scale = 1.0;
    };

    // Simple one-pole IIR filter with state
    struct OnePole {
        double y = 0.0;
        bool init = false;
        double filter(double x, double alpha) {
            if (!init) { y = x; init = true; return y; }
            y = alpha * x + (1.0 - alpha) * y;
            return y;
        }
        void reset() { y = 0.0; init = false; }
    };

    static double clampAlpha(double a) {
        if (a < 0.0) return 0.0;
        if (a > 1.0) return 1.0;
        return a;
    }

    void loadCalibration(const std::string &path) {
        calib_ = loadCalibrationFromFile(path);
    }

    static Calibration loadCalibrationFromFile(const std::string &path) {
        std::ifstream in(path);
        if (!in) throw std::runtime_error("ImuCalibrator: cannot open calibration file: " + path);
        json j; in >> j;
        Calibration c;
        c.accel_bias = j.at("accel_bias").get<std::array<double,3>>();
        c.gyro_bias  = j.at("gyro_bias").get<std::array<double,3>>();
        c.mag_bias   = j.at("mag_bias").get<std::array<double,3>>();
        c.mag_scale  = j.at("mag_scale").get<std::array<std::array<double,3>,3>>();
        c.accel_scale = j.value("accel_scale", 1.0);
        c.gyro_scale  = j.value("gyro_scale", 1.0);
        return c;
    }

    void resetFilters() {
        std::scoped_lock lk(filterMutex_);
        for (int i=0;i<3;i++) { filtAccel_[i].reset(); filtGyro_[i].reset(); filtMag_[i].reset(); }
    }

    // Members
    Calibration calib_;
    mutable std::shared_mutex calibMutex_; // allow concurrent reads, exclusive on reload
    OnePole filtAccel_[3];
    OnePole filtGyro_[3];
    OnePole filtMag_[3];
    std::mutex filterMutex_;               // protects filter states
    std::atomic<double> lpfAlpha_{0.5};
};


/*
How to use
#include "imu_calibrator.hpp"

// assume 'Device' is your driver with method AllAxes read_all();
Device dev; // already initialized
ImuCalibrator calib("mpu_calib.json", 0.4); // load calibration, set LPF alpha

// In your loop / thread where you read IMU:
while (running) {
    AllAxes raw = dev.read_all();            // returns raw sensor sample
    Calibrated filtered = calib.process(raw); // thread-safe call
    // use filtered values in your pipeline:
    // filtered.ax, filtered.ay, filtered.az (m/s^2)
    // filtered.gx, filtered.gy, filtered.gz (deg/s)
    // filtered.mx, filtered.my, filtered.mz (uT)
    // filtered.temp_c, filtered.timestamp
    robot_state.update_imu(filtered); // example consumer
}

#If you need to change LPF at runtime or reload calibration:
#process(...) is thread-safe; you can call it from real-time threads.
#Alpha=1.0 disables filtering (pass-through).

calib.setLpfAlpha(0.7);            // adjust filter responsiveness
calib.reloadCalibration("new_calib.json"); // hot-reload calibration

*/