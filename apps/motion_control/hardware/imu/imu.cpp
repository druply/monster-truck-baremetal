#include "imu.hpp"

#include <fcntl.h>
#include <unistd.h>
#include <cstring>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <chrono>
#include <thread>
#include <cmath>
#include <system_error>
#include <cerrno>
#include <array>
#include <numbers>
#include "LowPassFilter.hpp"
#include <iostream>

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

struct CalibrationData
{
    std::array<float, 3> accel_bias = {0.4757428938746452f, 0.32886819112300875f, -1.5145147708892814f};
    std::array<float, 3> gyro_bias = {-8.43256126022339f, -2.86207328128814f, -2.2680488500595093f};
    std::array<float, 3> mag_bias = {-25.357954025268555f, 5.069826889038086f, -151.2604949951172f};
    std::array<float, 3> mag_scale = {1.0401735414441589f, 0.9354390356732288f, 1.0313475955481064f};
    std::array<float, 3> accel_scale = {1.0f, 1.0f, 1.0f};
    std::array<float, 3> gyro_scale = {1.0f, 1.0f, 1.0f};
};

// Create a compile-time instance of your struct
inline constexpr CalibrationData default_calibration{};

void Imu::close()
{
    if (fd_i2c_ >= 0)
    {
        ::close(fd_i2c_);
        fd_i2c_ = -1;
    }
    initialized_ = false;
}

bool Imu::open_i2c()
{
    if (fd_i2c_ >= 0)
        return true;
    std::string path = "/dev/i2c-" + std::to_string(cfg_.i2c_bus);
    fd_i2c_ = ::open(path.c_str(), O_RDWR | O_CLOEXEC);
    return fd_i2c_ >= 0;
}

bool Imu::i2c_write_reg(uint8_t dev_addr, uint8_t reg, uint8_t val)
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

bool Imu::i2c_read_regs(uint8_t dev_addr, uint8_t reg, uint8_t *buf, size_t len)
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

float Imu::accel_fsr_to_scale(int fsr_g)
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

float Imu::gyro_fsr_to_scale(int fsr_dps)
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

int16_t Imu::be16(const uint8_t *b)
{
    return static_cast<int16_t>((b[0] << 8) | b[1]);
}

double Imu::monotonic_time_s()
{
    using namespace std::chrono;
    auto now = steady_clock::now();
    return duration<double>(now.time_since_epoch()).count();
}

// alternative for milliseconds
uint64_t Imu::monotonic_time_ms()
{
    using namespace std::chrono;
    auto now = steady_clock::now();
    return duration_cast<milliseconds>(now.time_since_epoch()).count();
}

std::error_code Imu::initialize()
{
    if (!open_i2c())
        return std::error_code(errno, std::generic_category());

    // reset device
    if (!i2c_write_reg(cfg_.mpu_addr, REG_PWR_MGMT_1, BIT_RESET))
        return std::make_error_code(std::errc::io_error);
    msleep(cfg_.init_delay_ms);

    // wake up and select clock source (auto selects best)
    if (!i2c_write_reg(cfg_.mpu_addr, REG_PWR_MGMT_1, 0x01))
        return std::make_error_code(std::errc::io_error);
    msleep(10);

    // disable standby axes
    if (!i2c_write_reg(cfg_.mpu_addr, REG_PWR_MGMT_2, 0x00))
        return std::make_error_code(std::errc::io_error);

    // verify WHO_AM_I
    uint8_t who = 0;
    if (!i2c_read_regs(cfg_.mpu_addr, REG_WHO_AM_I, &who, 1))
        return std::make_error_code(std::errc::io_error);
    // some clones differ; don't strictly require WHOAMI but warn by error code if mismatch
    if (who != WHO_AM_I_RESPONSE)
    {
        // still continue but return error
        // use EIO to indicate mismatch
        return std::error_code(EIO, std::generic_category());
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
        return std::make_error_code(std::errc::io_error);
    }

    initialized_ = true;
    filtered_data.ax = 0.0;
    filtered_data.ay = 0.0f;
    filtered_data.az = 0.0f;
    filtered_data.gx = 0.0f;
    filtered_data.gy = 0.0f;
    filtered_data.gz = 0.0f;
    filtered_data.mx = 0.0f;
    filtered_data.my = 0.0f;
    filtered_data.mz = 0.0f;
    filtered_data.heading = 0.0f;
    filtered_data.pitch = 0.0f;
    filtered_data.roll = 0.0f;
    filtered_data.delta_time = 0;
    return std::error_code(); // success
}

void Imu::apply_default_config()
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

bool Imu::setup_ak8963()
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

bool Imu::read_ak8963_adjustment()
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

bool Imu::read_ak8963_raw(int16_t out[3], uint8_t &st1)
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

std::optional<AllAxes> Imu::read_all()
{
    if (!initialized_)
        return std::nullopt;
    // read accel(6) temp(2) gyro(6) => 14 bytes starting at ACCEL_XOUT_H
    uint8_t buf[14] = {0};
    if (!i2c_read_regs(cfg_.mpu_addr, REG_ACCEL_XOUT_H, buf, sizeof(buf)))
        return std::nullopt;
    int16_t axr = be16(&buf[0]);
    int16_t ayr = be16(&buf[2]);
    int16_t azr = be16(&buf[4]);
    int16_t tr = be16(&buf[6]);
    int16_t gxr = be16(&buf[8]);
    int16_t gyr = be16(&buf[10]);
    int16_t gzr = be16(&buf[12]);

    // convert to physical units
    AllAxes out;
    out.ax = (float)axr * accel_scale_;
    out.ay = (float)ayr * accel_scale_;
    out.az = (float)azr * accel_scale_;
    out.gx = (float)gxr * gyro_scale_;
    out.gy = (float)gyr * gyro_scale_;
    out.gz = (float)gzr * gyro_scale_;
    out.temp_c = ((float)tr / TEMP_SENS) + TEMP_OFFSET;

    // read magnetometer via bypass
    uint8_t st1 = 0;
    int16_t mraw[3] = {0};
    if (read_ak8963_raw(mraw, st1))
    {
        out.mx = (float)mraw[0] * mag_scale_[0];
        out.my = (float)mraw[1] * mag_scale_[1];
        out.mz = (float)mraw[2] * mag_scale_[2];
    }
    else
    {
        // if failed, zero magnetometer
        out.mx = out.my = out.mz = NAN;
    }

    out.timestamp = monotonic_time_ms();
    return out;
}

Imu::Imu() : accel_filter(ACCEL_ALPHA_FACTOR), gyro_filter(GYRO_ALPHA_FACTOR), mag_filter(MAG_ALPHA_FACTOR)
{
}

Imu::~Imu() {}

void Imu::deInit(void) noexcept
{
    std::cout << "[Imu] deInit" << std::endl;
}
void Imu::run(void) noexcept
{
    std::cout << "[Imu] run" << std::endl;
    read_all_axis();
}
void Imu::init(void) noexcept
{
    std::cout << "[Imu] init" << std::endl;
    auto ec = initialize();
    if (ec)
    {
        std::cerr << "MPU9250 init failed: " << ec.message() << " (code " << ec.value() << ")\n";
        // return 1;
    }
}

void Imu::read_all(void)
{

    auto data = read_all();
    if (data)
    {
        // std::cout << "raw acc: " << data->ax << ", " << data->ay << ", " << data->az << std::endl;
        //  calculated corrected acceleration
        Vector3 raw_accel = {data->ax, data->ay, data->az};
        Vector3 raw_gyro = {data->gx, data->gy, data->gz};
        Vector3 raw_mag = {data->mx, data->my, data->mz};
        Vector3 corrected_accel;
        Vector3 corrected_gyro;
        Vector3 corrected_mag;
        Vector3 filtered_accel;
        Vector3 filtered_gyro;
        Vector3 filtered_mag;
        float roll_raw;
        float pitch_raw;
        float heading_raw;
        static uint64_t prev_timestamp;

        for (int i = 0; i < 3; ++i)
        {
            corrected_accel[i] = (raw_accel[i] - default_calibration.accel_bias[i]) * default_calibration.accel_scale[i];
            // filtered_accel[i] += ACCEL_ALPHA_FACTOR*(corrected_accel - filtered_accel[i]);
        }
        // std::cout << "corr acc: " << corrected_accel[0] << ", " << corrected_accel[1] << ", " << corrected_accel[2] << std::endl;

        for (int i = 0; i < 3; ++i)
        {
            corrected_gyro[i] = (raw_gyro[i] - default_calibration.gyro_bias[i]) * default_calibration.gyro_scale[i];
        }

        for (int i = 0; i < 3; ++i)
        {
            corrected_mag[i] = (raw_mag[i] - default_calibration.mag_bias[i]) * default_calibration.mag_scale[i];
        }

        // std::cout << "filtered acc: " << filtered_accel[0] << ", " << filtered_accel[1] << ", " << filtered_accel[2] << std::endl;
        // std::cout << "corr gyro: " << corrected_gyro[0] << ", " << corrected_gyro[1] << ", " << corrected_gyro[2] << std::endl;

        if (_first_run)
        {
            accel_filter.reset(corrected_accel);
            gyro_filter.reset(corrected_gyro);
            mag_filter.reset(corrected_mag);
            prev_timestamp = data->timestamp;
        }
        else
        {
            // filter acceleration data
            filtered_accel = accel_filter.update(corrected_accel);
            // filter gyro data
            filtered_gyro = gyro_filter.update(corrected_gyro);
            // filter gyro data
            filtered_mag = mag_filter.update(corrected_mag);
        }

        filtered_data.delta_time = data->timestamp - prev_timestamp;
        prev_timestamp = data->timestamp;

        // 1. Map to a standard right-handed NED-aligned coordinate frame
        float ax = -filtered_accel[0];
        float ay = -filtered_accel[1];
        float az = filtered_accel[2];

        float mx = -filtered_mag[0];
        float my = -filtered_mag[1];
        float mz = filtered_mag[2];

        // update imu data
        filtered_data.ax = ax;
        filtered_data.ay = ay;
        filtered_data.az = az;

        filtered_data.gx = -filtered_gyro[0];
        filtered_data.gy = -filtered_gyro[1];
        filtered_data.gz = filtered_gyro[2];

        filtered_data.mx = mx;
        filtered_data.my = my;
        filtered_data.mz = mz;

        std::cout << "imu ax: " << filtered_data.ax << std::endl;
        std::cout << "imu ay: " << filtered_data.ay << std::endl;
        std::cout << "imu az: " << filtered_data.az << std::endl;

        std::cout << "imu gx: " << filtered_data.gx << std::endl;
        std::cout << "imu gy: " << filtered_data.gy << std::endl;
        std::cout << "imu gz: " << filtered_data.gz << std::endl;

        std::cout << "imu mx: " << filtered_data.mx << std::endl;
        std::cout << "imu my: " << filtered_data.my << std::endl;
        std::cout << "imu mz: " << filtered_data.mz << std::endl;

        // 2. Calculate tilt compensation (Roll and Pitch)
        // Roll (phi) around the new X axis
        roll_raw = atan2(ay, az);

        // Pitch (theta) around the new Y axis
        pitch_raw = atan2(-ax, sqrt(ay * ay + az * az));

        // 3. Rotate magnetometer readings into the horizontal plane
        float cos_r = cos(roll_raw);
        float sin_r = sin(roll_raw);
        float cos_p = cos(pitch_raw);
        float sin_p = sin(pitch_raw);

        // Standard tilt-compensation rotation matrix equations
        float magx = mx * cos_p + my * sin_r * sin_p + mz * cos_r * sin_p;
        float magy = my * cos_r - mz * sin_r;

        // 4. Calculate heading and adjust for declination
        // Negating magy converts the counter-clockwise atan2 output to clockwise heading
        heading_raw = atan2(magy, magx) * 180.0f / pi_f;

        // Shift the reference frame by -90 degrees so 0 points North instead of East
        // heading_raw += 90.0f;

        // Wrap heading to a standard 0 to 360 degrees range
        heading_raw += DECLINATION;
        heading_raw += 10.0; // offset
        // if (heading_raw < 0.0f)   heading_raw += 360.0f;
        // if (heading_raw >= 360.0f) heading_raw -= 360.0f;
        std::cout << "heading_raw: " << heading_raw << std::endl;
        // // 3. Tilt Compensation
        // roll_raw = atan2(filtered_data.ay, filtered_data.az);
        // pitch_raw = atan2(-filtered_data.ax, sqrt(filtered_data.ay*filtered_data.ay + filtered_data.az*filtered_data.az));

        // float magX = filtered_data.mx * cos(pitch_raw) + filtered_data.mz * sin(pitch_raw);
        // float magY = filtered_data.mx * sin(roll_raw) * sin(pitch_raw) + filtered_data.my * cos(roll_raw) - filtered_data.mz * sin(roll_raw) * cos(pitch_raw);

        // // 4. Calculate Heading and add Declination
        // heading_raw = atan2(magY, magX) * 180.0f / pi_f;
        // heading_raw += DECLINATION;
        // filter heading angle
        // static float filtered_heading{0.0f};
        if (_first_run)
        {
            filtered_data.heading = heading_raw;
        }

        // filtered_heading +=HEADING_ALPHA_FACTOR*(heading_raw - filtered_heading);
        // filtered_data.heading += HEADING_ALPHA_FACTOR*(heading_raw - filtered_data.heading);
        float delta_time_s = static_cast<float>(filtered_data.delta_time) / 1000.0f;
        // calculate heading fusing with gyro
        filtered_data.heading = HEADING_ALPHA_FACTOR_FUSED * (filtered_data.heading + (filtered_data.gz * delta_time_s)) + (1.0 - HEADING_ALPHA_FACTOR_FUSED) * heading_raw;

        // set the first run to false only at the end of the first cycle
        if (_first_run)
        {
            _first_run = false;
        }
    }
}