#pragma once
#include "ModuleType.hpp"
#include <thread>
#include <optional>
#include "axis_struct.hpp"
#include "LowPassFilter.hpp"

struct Config {
    int i2c_bus = 1;                     // /dev/i2c-<bus>
    uint8_t mpu_addr = 0x68;             // MPU9250 default
    bool use_internal_ak8963 = true;     // if false, expects external magnetometer (not used)
    // Accelerometer FS: 2,4,8,16 g
    int accel_fsr_g = 16;
    // Gyro FS: 250,500,1000,2000 deg/s
    int gyro_fsr_dps = 2000;
    // DLPF bandwidth (Hz) typical: 184,92,41,20,10,5
    int dlpf_cfg = 3; // corresponds to about 44Hz depending on MCU datasheet; keep small
    // sample rate divider: sample_rate = 1000 / (1 + smplrt_div) when DLPF enabled
    uint8_t smplrt_div = 4;
    // number of retries for I2C transactions
    int i2c_retries = 3;
    // delay ms after reset/important ops
    int init_delay_ms = 100;
};

class LowPassFilter3D;

class Imu : public ModuleType{
 
        private:
                Config cfg_{};
                int fd_i2c_ = -1;
                bool initialized_ = false;

                // scale factors computed at init
                float accel_scale_ = 1.0f; // LSB->m/s^2
                float gyro_scale_ = 1.0f;  // LSB->deg/s
                float mag_scale_[3] = {1.0f,1.0f,1.0f}; // uT per LSB

                // raw read helpers
                bool i2c_write_reg(uint8_t dev_addr, uint8_t reg, uint8_t val);
                bool i2c_read_regs(uint8_t dev_addr, uint8_t reg, uint8_t* buf, size_t len);

                
          
                void close();
                // low-level i2c
                bool open_i2c();
                void apply_default_config();
                float accel_fsr_to_scale(int fsr_g);
                float gyro_fsr_to_scale(int fsr_dps);

                // AK8963 helpers
                bool setup_ak8963();
                bool read_ak8963_adjustment();
                bool read_ak8963_raw(int16_t out[3], uint8_t& st1);

                // utility
                static int16_t be16(const uint8_t* b);
                static double monotonic_time_s();
                static uint64_t monotonic_time_ms();
                std::optional<AllAxes> read_all();
               

                LowPassFilter3D accel_filter;
                LowPassFilter3D gyro_filter;
                LowPassFilter3D mag_filter;
                ImuData filtered_data{0};
                bool _first_run{true};
               

	public:
                Imu();
                ~Imu();
                void init(void) noexcept override;
                void run(void) noexcept override;
                void deInit(void) noexcept override;
                std::error_code initialize() ;
                 ImuData getImuData(void) {return filtered_data;}

                void read_all_axis(void);
                // Non-copyable, movable
                Imu(const Imu&) = delete;
                Imu& operator=(const Imu&) = delete;
                Imu(Imu&&) = default;
                Imu& operator=(Imu&&) = default;
};
