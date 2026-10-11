// src/drivers/pca9685_motor.cpp
#include "robot/drivers/pca9685_motor.hpp"

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <cstring>
#include <cmath>
#include <algorithm>

namespace robot::drivers {

namespace {

// PCA9685 registers
constexpr std::uint8_t REG_MODE1         = 0x00;
constexpr std::uint8_t REG_MODE2         = 0x01;
constexpr std::uint8_t REG_LED0_ON_L     = 0x06;
constexpr std::uint8_t REG_PRESCALE      = 0xFE;

// MODE1 bits
constexpr std::uint8_t MODE1_RESTART = 0x80;
constexpr std::uint8_t MODE1_SLEEP   = 0x10;
constexpr std::uint8_t MODE1_AI      = 0x20;   // auto-increment

// MODE2 bits
constexpr std::uint8_t MODE2_OUTDRV  = 0x04;   // totem-pole output

constexpr int PWM_RESOLUTION = 4096;

} // namespace

struct Pca9685Motor::Impl {
    Pca9685MotorConfig cfg;
    int fd{-1};
    bool healthy{false};

    Result<void> write_reg(std::uint8_t reg, std::uint8_t val) {
        std::uint8_t buf[2] = {reg, val};
        if (::write(fd, buf, 2) != 2) return std::unexpected(Error::IoFailure);
        return {};
    }

    Result<std::uint8_t> read_reg(std::uint8_t reg) {
        if (::write(fd, &reg, 1) != 1) return std::unexpected(Error::IoFailure);
        std::uint8_t val{};
        if (::read(fd, &val, 1) != 1) return std::unexpected(Error::IoFailure);
        return val;
    }

    // Set a channel's PWM: ON=0, OFF=duty (0..4095)
    Result<void> set_channel_duty(std::uint8_t ch, std::uint16_t duty) {
        if (duty > PWM_RESOLUTION) duty = PWM_RESOLUTION;
        std::uint8_t base = static_cast<std::uint8_t>(REG_LED0_ON_L + 4 * ch);

        std::uint8_t buf[5] = {
            base,
            0x00,                                          // ON_L
            0x00,                                          // ON_H
            static_cast<std::uint8_t>(duty & 0xFF),        // OFF_L
            static_cast<std::uint8_t>((duty >> 8) & 0x0F)  // OFF_H (low 4 bits)
        };
        if (::write(fd, buf, 5) != 5) return std::unexpected(Error::IoFailure);
        return {};
    }

    Result<void> set_pwm_frequency(int freq_hz) {
        if (freq_hz < 24 || freq_hz > 1526) {
            return std::unexpected(Error::InvalidArgument);
        }

        double prescale_f = 25'000'000.0 / (PWM_RESOLUTION * freq_hz) - 1.0;
        auto prescale = static_cast<std::uint8_t>(std::round(prescale_f));

        // Must enter SLEEP before writing PRESCALE
        auto mode1 = read_reg(REG_MODE1);
        if (!mode1) return std::unexpected(mode1.error());

        std::uint8_t sleep_val = static_cast<std::uint8_t>((*mode1) | MODE1_SLEEP);
        if (auto r = write_reg(REG_MODE1, sleep_val); !r) return r;
        usleep(500);  // wait for oscillator to stop

        if (auto r = write_reg(REG_PRESCALE, prescale); !r) return r;

        // Wake up
        std::uint8_t wake_val = static_cast<std::uint8_t>((*mode1) & ~MODE1_SLEEP);
        if (auto r = write_reg(REG_MODE1, wake_val); !r) return r;
        usleep(500);  // wait for oscillator to stabilize

        // Trigger RESTART in case PWM was already running
        if (auto r = write_reg(REG_MODE1, wake_val | MODE1_RESTART); !r) return r;

        return {};
    }
};

Pca9685Motor::Pca9685Motor(Pca9685MotorConfig cfg)
    : impl_(std::make_unique<Impl>(Impl{std::move(cfg)})) {}

Pca9685Motor::~Pca9685Motor() {
    if (impl_->fd >= 0) {
        // Fail-safe: force channel to 0
        (void)impl_->set_channel_duty(impl_->cfg.pwm_channel, 0);
        ::close(impl_->fd);
    }
}

Result<void> Pca9685Motor::initialize() {
    impl_->fd = ::open(impl_->cfg.i2c_device.c_str(), O_RDWR);
    if (impl_->fd < 0) return std::unexpected(Error::DeviceNotFound);

    if (::ioctl(impl_->fd, I2C_SLAVE, impl_->cfg.address) < 0)
        return std::unexpected(Error::DeviceNotFound);

    // Verify device: reading MODE1 should succeed
    if (auto r = impl_->read_reg(REG_MODE1); !r)
        return std::unexpected(Error::DeviceNotFound);

    // MODE1: enable AI (auto-increment), not sleeping, not restarting
    if (auto r = impl_->write_reg(REG_MODE1,
                                  static_cast<std::uint8_t>(MODE1_AI & ~MODE1_SLEEP)); !r)
        return r;

    // MODE2: totem-pole output
    if (auto r = impl_->write_reg(REG_MODE2, MODE2_OUTDRV); !r)
        return r;

    // Set PWM frequency
    if (auto r = impl_->set_pwm_frequency(impl_->cfg.pwm_frequency_hz); !r)
        return r;

    // Safe initial state
    if (auto r = impl_->set_channel_duty(impl_->cfg.pwm_channel, 0); !r)
        return r;

    if (impl_->cfg.dir_channel_active) {
        // Direction channel initially low (forward)
        if (auto r = impl_->set_channel_duty(impl_->cfg.dir_channel, 0); !r)
            return r;
    }

    impl_->healthy = true;
    return {};
}

Result<void> Pca9685Motor::set(const MotorCommand& cmd) {
    if (!impl_->healthy) return std::unexpected(Error::NotInitialized);

    double duty = std::clamp(cmd.duty, -1.0, 1.0);
    if (impl_->cfg.invert) duty = -duty;

    // Direction channel: low (0) for forward, high (4095) for reverse
    if (impl_->cfg.dir_channel_active) {
        std::uint16_t dir_val = (duty >= 0.0) ? 0 : PWM_RESOLUTION;
        if (auto r = impl_->set_channel_duty(impl_->cfg.dir_channel, dir_val); !r)
            return r;
    }

    // Enable channel: duty = |duty| * 4095
    std::uint16_t pwm_val = static_cast<std::uint16_t>(
        std::abs(duty) * (PWM_RESOLUTION - 1));
    if (auto r = impl_->set_channel_duty(impl_->cfg.pwm_channel, pwm_val); !r) {
        impl_->healthy = false;
        return std::unexpected(r.error());
    }

    return {};
}

Result<void> Pca9685Motor::stop() {
    return set(MotorCommand{.duty = 0.0});
}

bool Pca9685Motor::is_healthy() const noexcept {
    return impl_->healthy;
}

} // namespace robot::drivers