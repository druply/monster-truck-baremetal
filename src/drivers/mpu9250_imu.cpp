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

        // MPU9250 registers (core)
        constexpr uint8_t REG_PWR_MGMT_1 = 0x6B;
        constexpr uint8_t REG_PWR_MGMT_2 = 0x6C;
        constexpr uint8_t REG_WHO_AM_I = 0x75;
        constexpr uint8_t REG_SMPLRT_DIV = 0x19;
        constexpr uint8_t REG_CONFIG = 0x1A;
        constexpr uint8_t REG_GYRO_CFG = 0x1B;
        constexpr uint8_t REG_ACCEL_CFG = 0x1C;
        constexpr uint8_t REG_ACCEL_CFG2 = 0x1D;
        constexpr uint8_t REG_INT_PIN_CFG = 0x37;
        constexpr uint8_t REG_INT_ENABLE = 0x38;
        constexpr uint8_t REG_ACCEL_XOUT_H = 0x3B;
        constexpr uint8_t REG_TEMP_OUT_H = 0x41;
        constexpr uint8_t REG_GYRO_XOUT_H = 0x43;

        // AK8963 (mag) registers
        constexpr uint8_t AK8963_I2C_ADDR = 0x0C;
        constexpr uint8_t AK_REG_WIA = 0x00;
        constexpr uint8_t AK_REG_CNTL1 = 0x0A;
        constexpr uint8_t AK_REG_ASAX = 0x10;
        constexpr uint8_t AK_REG_ST1 = 0x02;
        constexpr uint8_t AK_REG_HXL = 0x03;
        constexpr uint8_t AK_REG_ST2 = 0x09;

        // MPU9250 USER_CTRL / I2C_MST for bypass/passthrough
        constexpr uint8_t REG_USER_CTRL = 0x6A;
        constexpr uint8_t REG_I2C_MST_CTRL = 0x24;
        constexpr uint8_t REG_I2C_SLV0_ADDR = 0x25;
        constexpr uint8_t REG_I2C_SLV0_REG = 0x26;
        constexpr uint8_t REG_I2C_SLV0_CTRL = 0x27;
        constexpr uint8_t REG_EXT_SENS_DATA_00 = 0x49;

        // bits
        constexpr uint8_t BIT_RESET = 0x80;
        constexpr uint8_t BIT_BYPASS_EN = 0x02;
        constexpr uint8_t BIT_I2C_MST_EN = 0x20;

        // expected WHO_AM_I
        constexpr uint8_t WHO_AM_I_RESPONSE = 0x71; // MPU9250 typical

        // constants
        constexpr float G = 9.80665f;
        constexpr float TEMP_SENS = 333.87f; // LSB per deg C ? datasheet uses: temp = (raw / 333.87) + 21
        constexpr float TEMP_OFFSET = 21.0f;
        const float DECLINATION = -6.3f;
        constexpr float pi_f = std::numbers::pi_v<float>;
        constexpr float MAG_ALPHA_FACTOR = 0.1f;
        constexpr float GYRO_ALPHA_FACTOR = 0.1f;
        constexpr float ACCEL_ALPHA_FACTOR = 0.1f;
        constexpr float HEADING_ALPHA_FACTOR_FUSED = 0.7f;
        constexpr float HEADING_ALPHA_FACTOR = 0.1f;

        // helpers
        inline void msleep(int ms) { std::this_thread::sleep_for(std::chrono::milliseconds(ms)); }

    } // namespace

    void Mpu9250Imu::close()
    {
        if (fd_i2c_ >= 0)
        {
            ::close(fd_i2c_);
            fd_i2c_ = -1;
        }
        initialized_ = false;
    }

    bool Mpu9250Imu::open_i2c()
    {
        if (fd_i2c_ >= 0)
            return true;
        std::string path = "/dev/i2c-" + std::to_string(cfg_.i2c_bus);
        fd_i2c_ = ::open(path.c_str(), O_RDWR | O_CLOEXEC);
        return fd_i2c_ >= 0;
    }

    bool Mpu9250Imu::i2c_write_reg(uint8_t dev_addr, uint8_t reg, uint8_t val)
    {
        if (!open_i2c())
            return false;
        if (ioctl(fd_i2c_, I2C_SLAVE, dev_addr) < 0)
            return false;
        uint8_t buf[2] = {reg, val};
        for (int i = 0; i < cfg_.i2c_retries; i++)
        {
            ssize_t w = ::write(fd_i2c_, buf, 2);
            if (w == 2)
                return true;
            msleep(1);
        }
        return false;
    }

    bool Mpu9250Imu::i2c_read_regs(uint8_t dev_addr, uint8_t reg, uint8_t *buf, size_t len)
    {
        if (!open_i2c())
            return false;
        if (ioctl(fd_i2c_, I2C_SLAVE, dev_addr) < 0)
            return false;
        // write register
        for (int i = 0; i < cfg_.i2c_retries; i++)
        {
            ssize_t w = ::write(fd_i2c_, &reg, 1);
            if (w == 1)
                break;
            msleep(1);
        }
        // read
        for (int i = 0; i < cfg_.i2c_retries; i++)
        {
            ssize_t r = ::read(fd_i2c_, buf, len);
            if (r == (ssize_t)len)
                return true;
            msleep(1);
        }
        return false;
    }

    float Mpu9250Imu::accel_fsr_to_scale(int fsr_g)
    {
        // LSB per g: FS_SEL=0->16384 (2g),1->8192,2->4096,3->2048 (16g)
        switch (fsr_g)
        {
        case 2:
            return (G / 16384.0f);
        case 4:
            return (G / 8192.0f);
        case 8:
            return (G / 4096.0f);
        case 16:
            return (G / 2048.0f);
        default:
            return (G / 2048.0f);
        }
    }

    float Mpu9250Imu::gyro_fsr_to_scale(int fsr_dps)
    {
        // LSB per deg/s: 250->131,500->65.5,1000->32.8,2000->16.4
        switch (fsr_dps)
        {
        case 250:
            return (1.0f / 131.0f);
        case 500:
            return (1.0f / 65.5f);
        case 1000:
            return (1.0f / 32.8f);
        case 2000:
            return (1.0f / 16.4f);
        default:
            return (1.0f / 16.4f);
        }
    }

    int16_t Mpu9250Imu::be16(const uint8_t *b)
    {
        return static_cast<int16_t>((b[0] << 8) | b[1]);
    }

    // double Mpu9250Imu::monotonic_time_s()
    // {
    //     using namespace std::chrono;
    //     auto now = steady_clock::now();
    //     return duration<double>(now.time_since_epoch()).count();
    // }

    // // alternative for milliseconds
    // Timestamp Mpu9250Imu::monotonic_time_ms()
    // {
    //     using namespace std::chrono;
    //     auto now = steady_clock::now();
    //     return duration_cast<milliseconds>(now.time_since_epoch()).count();
    // }

    void Mpu9250Imu::apply_default_config()
    {
        // sample rate divider
        i2c_write_reg(cfg_.mpu_addr, REG_SMPLRT_DIV, cfg_.smplrt_div);
        // DLPF / CONFIG
        i2c_write_reg(cfg_.mpu_addr, REG_CONFIG, cfg_.dlpf_cfg);
        // gyro config (FS_SEL in bits 4:3)
        uint8_t gyro_cfg = 0;
        switch (cfg_.gyro_fsr_dps)
        {
        case 250:
            gyro_cfg = 0x00;
            break;
        case 500:
            gyro_cfg = 0x08;
            break;
        case 1000:
            gyro_cfg = 0x10;
            break;
        case 2000:
            gyro_cfg = 0x18;
            break;
        default:
            gyro_cfg = 0x18;
            break;
        }
        i2c_write_reg(cfg_.mpu_addr, REG_GYRO_CFG, gyro_cfg);
        // accel config (AFS_SEL bits 4:3)
        uint8_t accel_cfg = 0;
        switch (cfg_.accel_fsr_g)
        {
        case 2:
            accel_cfg = 0x00;
            break;
        case 4:
            accel_cfg = 0x08;
            break;
        case 8:
            accel_cfg = 0x10;
            break;
        case 16:
            accel_cfg = 0x18;
            break;
        default:
            accel_cfg = 0x18;
            break;
        }
        i2c_write_reg(cfg_.mpu_addr, REG_ACCEL_CFG, accel_cfg);
        // accel config2: set DLPF for accel
        i2c_write_reg(cfg_.mpu_addr, REG_ACCEL_CFG2, cfg_.dlpf_cfg);
        // INT pin / bypassenable for magnetometer: set bypass bit
        i2c_write_reg(cfg_.mpu_addr, REG_INT_PIN_CFG, BIT_BYPASS_EN);
        // enable data ready interrupt optionally
        i2c_write_reg(cfg_.mpu_addr, REG_INT_ENABLE, 0x01);
    }

    bool Mpu9250Imu::setup_ak8963()
    {
        // Ensure bypass enabled so we can directly access AK8963 at 0x0C
        if (!i2c_write_reg(cfg_.mpu_addr, REG_INT_PIN_CFG, BIT_BYPASS_EN))
            return false;
        msleep(10);
        // check AK8963 WHOAMI
        uint8_t who = 0;
        if (!i2c_read_regs(AK8963_I2C_ADDR, AK_REG_WIA, &who, 1))
            return false;
        if (who != 0x48)
            return false; // AK8963 WIA expected 0x48

        // Power down magnetometer
        if (!i2c_write_reg(AK8963_I2C_ADDR, AK_REG_CNTL1, 0x00))
            return false;
        msleep(10);
        // Enter fuse ROM access mode to read sensitivity adjustments
        if (!i2c_write_reg(AK8963_I2C_ADDR, AK_REG_CNTL1, 0x0F))
            return false;
        msleep(10);
        if (!read_ak8963_adjustment())
            return false;
        // power down then set continuous measurement 16-bit 100Hz (0x16)
        if (!i2c_write_reg(AK8963_I2C_ADDR, AK_REG_CNTL1, 0x00))
            return false;
        msleep(10);
        // 16-bit continuous measurement mode 2 -> 100Hz: CNTL1=0x16 (0b00010110)
        if (!i2c_write_reg(AK8963_I2C_ADDR, AK_REG_CNTL1, 0x16))
            return false;
        msleep(10);
        return true;
    }

    bool Mpu9250Imu::read_ak8963_adjustment()
    {
        uint8_t asa[3] = {0};
        if (!i2c_read_regs(AK8963_I2C_ADDR, AK_REG_ASAX, asa, 3))
            return false;
        // conversion: mag sensitivity adjustment: (ASA - 128)/256 + 1
        for (int i = 0; i < 3; i++)
        {
            float adj = ((float)asa[i] - 128.0f) / 256.0f + 1.0f;
            // AK8963 16-bit LSB per uT ~ 0.15? We'll derive scale: typical raw->uT factor ~ 0.15 uT/LSB for 16-bit
            // Standard: 4912 uT / 32760 LSB = 0.150 (approx) per LSB. Use 4912/32760 = 0.1500
            mag_scale_[i] = adj * (4912.0f / 32760.0f); // uT per LSB
        }
        return true;
    }

    bool Mpu9250Imu::read_ak8963_raw(int16_t out[3], uint8_t &st1)
    {
        // read ST1
        uint8_t st = 0;
        if (!i2c_read_regs(AK8963_I2C_ADDR, AK_REG_ST1, &st, 1))
            return false;
        st1 = st;
        if (!(st & 0x01))
            return false; // data not ready
        uint8_t buff[7] = {0};
        if (!i2c_read_regs(AK8963_I2C_ADDR, AK_REG_HXL, buff, 7))
            return false;
        // check overflow ST2 bit
        uint8_t st2 = buff[6];
        if (st2 & 0x08)
            return false;
        // little-endian for magnetometer
        out[0] = static_cast<int16_t>((uint16_t)buff[1] << 8 | buff[0]);
        out[1] = static_cast<int16_t>((uint16_t)buff[3] << 8 | buff[2]);
        out[2] = static_cast<int16_t>((uint16_t)buff[5] << 8 | buff[4]);
        return true;
    }

    Result<void> Mpu9250Imu::initialize()
    {
        if (!open_i2c())
            // return std::error_code(errno, std::generic_category());
            return std::unexpected(Error{
                Module::Imu, ErrorCode::IoFailure, "Failed to open I2C device"});

        // reset device
        if (!i2c_write_reg(cfg_.mpu_addr, REG_PWR_MGMT_1, BIT_RESET))
            // return std::make_error_code(std::errc::io_error);
            return std::unexpected(Error{
                Module::Imu, ErrorCode::IoFailure, "Failed to write to PWR_MGMT_1"});
        msleep(cfg_.init_delay_ms);

        // wake up and select clock source (auto selects best)
        if (!i2c_write_reg(cfg_.mpu_addr, REG_PWR_MGMT_1, 0x01))
            // return std::make_error_code(std::errc::io_error);
            return std::unexpected(Error{
                Module::Imu, ErrorCode::IoFailure, "Failed to write to PWR_MGMT_1"});
        msleep(10);

        // disable standby axes
        if (!i2c_write_reg(cfg_.mpu_addr, REG_PWR_MGMT_2, 0x00))
            // return std::make_error_code(std::errc::io_error);
            return std::unexpected(Error{
                Module::Imu, ErrorCode::IoFailure, "Failed to write to PWR_MGMT_2"});

        // verify WHO_AM_I
        uint8_t who = 0;
        if (!i2c_read_regs(cfg_.mpu_addr, REG_WHO_AM_I, &who, 1))
            return std::unexpected(Error{
                Module::Imu, ErrorCode::IoFailure, "Failed to read WHO_AM_I"});
        // some clones differ; don't strictly require WHOAMI but warn by error code if mismatch
        if (who != WHO_AM_I_RESPONSE)
        {
            // still continue but return error
            // use EIO to indicate mismatch
            return std::unexpected(Error{
                Module::Imu, ErrorCode::IoFailure, "WHO_AM_I mismatch"});
        }

        apply_default_config();

        // compute scales
        accel_scale_ = accel_fsr_to_scale(cfg_.accel_fsr_g);
        gyro_scale_ = gyro_fsr_to_scale(cfg_.gyro_fsr_dps);

        // setup magnetometer
        if (!setup_ak8963())
        {
            // allow non-fatal: if magnetometer not available, still ok but indicate error
            // we choose to return error
            return std::unexpected(Error{
                Module::Imu, ErrorCode::DeviceNotFound, "Failed to setup AK8963 magnetometer"});
        }

        //initialized_ = true;
        // filtered_data.ax = 0.0;
        // filtered_data.ay = 0.0f;
        // filtered_data.az = 0.0f;
        // filtered_data.gx = 0.0f;
        // filtered_data.gy = 0.0f;
        // filtered_data.gz = 0.0f;
        // filtered_data.mx = 0.0f;
        // filtered_data.my = 0.0f;
        // filtered_data.mz = 0.0f;
        // filtered_data.heading = 0.0f;
        // filtered_data.pitch = 0.0f;
        // filtered_data.roll = 0.0f;
        // filtered_data.delta_time = 0;
        // return std::error_code(); // success
    }

        Result<ImuSample> Mpu9250Imu::read()
    {
        if (!initialized_)
            return std::nullopt;
        // read accel(6) temp(2) gyro(6) => 14 bytes starting at ACCEL_XOUT_H
        uint8_t buf[14] = {0};
        if (!i2c_read_regs(cfg_.mpu_addr, REG_ACCEL_XOUT_H, buf, sizeof(buf)))
        {
            m_healthy = false;
            return std::unexpected(Error{
                Module::Imu, ErrorCode::IoFailure, "Failed to read IMU registers"});
        }

        int16_t axr = be16(&buf[0]);
        int16_t ayr = be16(&buf[2]);
        int16_t azr = be16(&buf[4]);
        int16_t tr = be16(&buf[6]);
        int16_t gxr = be16(&buf[8]);
        int16_t gyr = be16(&buf[10]);
        int16_t gzr = be16(&buf[12]);

        // convert to physical units
        ImuSample out;
        out.accel.x = (float)axr * accel_scale_;
        out.accel.y = (float)ayr * accel_scale_;
        out.accel.z = (float)azr * accel_scale_;
        out.gyro.x = (float)gxr * gyro_scale_;
        out.gyro.y = (float)gyr * gyro_scale_;
        out.gyro.z = (float)gzr * gyro_scale_;
        out.temp_c = ((float)tr / TEMP_SENS) + TEMP_OFFSET;

        // read magnetometer via bypass
        uint8_t st1 = 0;
        int16_t mraw[3] = {0};
        if (read_ak8963_raw(mraw, st1))
        {
            out.mag.x = (float)mraw[0] * mag_scale_[0];
            out.mag.y = (float)mraw[1] * mag_scale_[1];
            out.mag.z = (float)mraw[2] * mag_scale_[2];
        }
        else
        {
            // if failed, zero magnetometer
            out.mag.x = out.mag.y = out.mag.z = NAN;
            m_healthy = false;
        }

        out.timestamp = m_clock.monotonic_time_ms();
        m_healthy = true;
        return out;
    }

    bool Mpu9250Imu::is_healthy() const noexcept
    {
        return m_healthy;
    }

} // namespace robot::drivers