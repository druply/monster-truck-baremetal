// src/drivers/mpu9250_imu.cpp
#include "robot/drivers/mpu9250_imu.hpp"

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <cstring>
#include <cmath>
#include <array>

namespace robot::drivers
{

    // ============ Register constants (file-local) ============
    namespace
    {

        // MPU-6500 side
        constexpr std::uint8_t REG_SMPLRT_DIV = 0x19;
        constexpr std::uint8_t REG_CONFIG = 0x1A;
        constexpr std::uint8_t REG_GYRO_CONFIG = 0x1B;
        constexpr std::uint8_t REG_ACCEL_CONFIG = 0x1C;
        constexpr std::uint8_t REG_INT_PIN_CFG = 0x37;
        constexpr std::uint8_t REG_ACCEL_XOUT_H = 0x3B;
        constexpr std::uint8_t REG_GYRO_XOUT_H = 0x43;
        constexpr std::uint8_t REG_PWR_MGMT_1 = 0x6B;
        constexpr std::uint8_t REG_WHO_AM_I = 0x75;

        // AK8963 magnetometer (independent I²C address, visible after bypass)
        constexpr std::uint8_t AK8963_ADDR = 0x0C;
        constexpr std::uint8_t AK8963_WIA = 0x00; // should read 0x48
        constexpr std::uint8_t AK8963_ST1 = 0x02;
        constexpr std::uint8_t AK8963_HXL = 0x03; // data start
        constexpr std::uint8_t AK8963_CNTL1 = 0x0A;
        constexpr std::uint8_t AK8963_CNTL2 = 0x0B;
        constexpr std::uint8_t AK8963_ASAX = 0x10; // sensitivity adjustment

        // Bit masks
        constexpr std::uint8_t PWR_WAKE = 0x00;
        constexpr std::uint8_t BYPASS_EN = 0x02;
        constexpr std::uint8_t AK8963_16BIT = 0x10;
        constexpr std::uint8_t AK8963_CONT_100HZ = 0x16; // continuous mode 2, 100 Hz
        constexpr std::uint8_t AK8963_RESET = 0x01;

        // Full-scale sensitivities (LSB per unit)
        constexpr double ACCEL_SENS_8G = 4096.0; // LSB/g
        constexpr double GYRO_SENS_1000 = 32.8;  // LSB/(°/s)
        constexpr double TEMP_SENS = 333.87;     // LSB/°C
        constexpr double TEMP_OFFSET = 21.0;

        constexpr double G_TO_MS2 = 9.80665;
        constexpr double DEG_TO_RAD = 3.14159265358979323846 / 180.0;

    } // namespace

    // ============ Pimpl ============
    struct Mpu9250Imu::Impl
    {
        Mpu9250Config cfg;
        std::shared_ptr<hal::IClock> clock;
        int fd{-1};
        bool healthy{false};

        // AK8963 sensitivity adjustment factors
        std::array<double, 3> mag_adj{1.0, 1.0, 1.0};

        // ---- Raw I²C primitives ----
        Result<void> write_reg(std::uint8_t addr, std::uint8_t reg, std::uint8_t val)
        {
            if (::ioctl(fd, I2C_SLAVE, addr) < 0)
                return std::unexpected(Error::IoFailure);
            std::uint8_t buf[2] = {reg, val};
            if (::write(fd, buf, 2) != 2)
                return std::unexpected(Error::IoFailure);
            return {};
        }

        Result<std::uint8_t> read_reg(std::uint8_t addr, std::uint8_t reg)
        {
            if (::ioctl(fd, I2C_SLAVE, addr) < 0)
                return std::unexpected(Error::IoFailure);
            if (::write(fd, &reg, 1) != 1)
                return std::unexpected(Error::IoFailure);
            std::uint8_t val{};
            if (::read(fd, &val, 1) != 1)
                return std::unexpected(Error::IoFailure);
            return val;
        }

        Result<void> read_burst(std::uint8_t addr, std::uint8_t reg,
                                std::uint8_t *buf, std::size_t len)
        {
            if (::ioctl(fd, I2C_SLAVE, addr) < 0)
                return std::unexpected(Error::IoFailure);
            if (::write(fd, &reg, 1) != 1)
                return std::unexpected(Error::IoFailure);
            if (::read(fd, buf, len) != static_cast<ssize_t>(len))
                return std::unexpected(Error::IoFailure);
            return {};
        }

        // ---- Init steps ----
        Result<void> init_mpu()
        {
            // Wake up, use PLL clock
            if (auto r = write_reg(cfg.mpu_address, REG_PWR_MGMT_1, PWR_WAKE); !r)
                return r;
            usleep(10000);

            // Verify WHO_AM_I (MPU-9250 reads 0x71)
            auto who = read_reg(cfg.mpu_address, REG_WHO_AM_I);
            if (!who || *who != 0x71)
                return std::unexpected(Error::DeviceNotFound);

            // Sample rate divider: 1 kHz / (1 + div)
            auto div = static_cast<std::uint8_t>(1000 / cfg.sample_rate_hz - 1);
            if (auto r = write_reg(cfg.mpu_address, REG_SMPLRT_DIV, div); !r)
                return r;

            // DLPF: gyro BW 41 Hz, accel 44.8 Hz
            if (auto r = write_reg(cfg.mpu_address, REG_CONFIG, 0x03); !r)
                return r;

            // Gyro full scale
            auto gyro_cfg = (cfg.gyro_range_dps == 250) ? 0x00 : (cfg.gyro_range_dps == 500) ? 0x08
                                                             : (cfg.gyro_range_dps == 1000)  ? 0x10
                                                                                             : 0x18;
            if (auto r = write_reg(cfg.mpu_address, REG_GYRO_CONFIG, gyro_cfg); !r)
                return r;

            // Accel full scale
            auto accel_cfg = (cfg.accel_range_g == 2) ? 0x00 : (cfg.accel_range_g == 4) ? 0x08
                                                           : (cfg.accel_range_g == 8)   ? 0x10
                                                                                        : 0x18;
            if (auto r = write_reg(cfg.mpu_address, REG_ACCEL_CONFIG, accel_cfg); !r)
                return r;

            return {};
        }

        Result<void> init_ak8963_bypass()
        {
            // Enable I²C bypass so the AK8963 appears on the external bus
            if (auto r = write_reg(cfg.mpu_address, REG_INT_PIN_CFG, BYPASS_EN); !r)
                return r;
            usleep(1000);

            // Reset magnetometer
            if (auto r = write_reg(AK8963_ADDR, AK8963_CNTL2, AK8963_RESET); !r)
                return r;
            usleep(10000);

            // Verify AK8963 ID
            auto wia = read_reg(AK8963_ADDR, AK8963_WIA);
            if (!wia || *wia != 0x48)
                return std::unexpected(Error::DeviceNotFound);

            // Read sensitivity adjustment (requires FUSE ROM access mode first)
            if (auto r = write_reg(AK8963_ADDR, AK8963_CNTL1, 0x0F); !r)
                return r;
            usleep(1000);

            std::uint8_t asa[3]{};
            if (auto r = read_burst(AK8963_ADDR, AK8963_ASAX, asa, 3); !r)
                return r;
            for (int i = 0; i < 3; ++i)
            {
                mag_adj[i] = (static_cast<double>(asa[i]) - 128.0) / 256.0 + 1.0;
            }

            // Continuous mode 2, 16-bit output, 100 Hz
            if (auto r = write_reg(AK8963_ADDR, AK8963_CNTL1,
                                   AK8963_16BIT | AK8963_CONT_100HZ);
                !r)
                return r;
            usleep(10000);

            return {};
        }
    };

    // ============ Public interface ============

    Mpu9250Imu::Mpu9250Imu(Mpu9250Config cfg, std::shared_ptr<hal::IClock> clock)
        : impl_(std::make_unique<Impl>(Impl{std::move(cfg), std::move(clock)})) {}

    Mpu9250Imu::~Mpu9250Imu()
    {
        if (impl_->fd >= 0)
            ::close(impl_->fd);
    }

    Result<void> Mpu9250Imu::initialize()
    {
        impl_->fd = ::open(impl_->cfg.i2c_device.c_str(), O_RDWR);
        if (impl_->fd < 0)
            return std::unexpected(Error::DeviceNotFound);

        if (auto r = impl_->init_mpu(); !r)
            return r;

        if (impl_->cfg.enable_magnetometer)
        {
            if (auto r = impl_->init_ak8963_bypass(); !r)
                return r;
        }

        impl_->healthy = true;
        return {};
    }

    Result<ImuSample> Mpu9250Imu::read()
    {
        if (!impl_->healthy)
            return std::unexpected(Error::NotInitialized);

        // Single 14-byte read: ACCEL(6) + TEMP(2) + GYRO(6)
        std::uint8_t raw[14]{};
        if (auto r = impl_->read_burst(impl_->cfg.mpu_address,
                                       REG_ACCEL_XOUT_H, raw, 14);
            !r)
        {
            impl_->healthy = false;
            return std::unexpected(r.error());
        }

        auto be16 = [](const std::uint8_t *p) -> std::int16_t
        {
            return static_cast<std::int16_t>((p[0] << 8) | p[1]);
        };

        ImuSample s;
        s.stamp = impl_->clock->now();

        // Accel: LSB -> g -> m/s²
        s.accel.x = (be16(&raw[0]) / ACCEL_SENS_8G) * G_TO_MS2;
        s.accel.y = (be16(&raw[2]) / ACCEL_SENS_8G) * G_TO_MS2;
        s.accel.z = (be16(&raw[4]) / ACCEL_SENS_8G) * G_TO_MS2;

        // Temperature
        s.temperature_c = (be16(&raw[6]) / TEMP_SENS) + TEMP_OFFSET;

        // Gyro: LSB -> °/s -> rad/s
        s.gyro.x = (be16(&raw[8]) / GYRO_SENS_1000) * DEG_TO_RAD;
        s.gyro.y = (be16(&raw[10]) / GYRO_SENS_1000) * DEG_TO_RAD;
        s.gyro.z = (be16(&raw[12]) / GYRO_SENS_1000) * DEG_TO_RAD;

        // Magnetometer (AK8963)
        if (impl_->cfg.enable_magnetometer)
        {
            std::uint8_t mag_raw[7]{}; // 6 data + ST2 handshake
            if (auto mr = impl_->read_burst(AK8963_ADDR, AK8963_HXL, mag_raw, 7); mr)
            {
                auto le16 = [](const std::uint8_t *p) -> std::int16_t
                {
                    return static_cast<std::int16_t>((p[1] << 8) | p[0]);
                };
                // AK8963 is little-endian; raw counts scaled by adjustment factors
                s.mag.x = le16(&mag_raw[0]) * impl_->mag_adj[0];
                s.mag.y = le16(&mag_raw[2]) * impl_->mag_adj[1];
                s.mag.z = le16(&mag_raw[4]) * impl_->mag_adj[2];
            }
        }

        return s;
    }

    bool Mpu9250Imu::is_healthy() const noexcept
    {
        return impl_->healthy;
    }

} // namespace robot::drivers