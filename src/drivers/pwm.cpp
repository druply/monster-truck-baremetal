#include "robot/drivers/pwm.hpp"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <string>
#include <thread>
#include <utility>

#if defined(__linux__)
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>
#endif

namespace robot::drivers
{
namespace
{
#if defined(__linux__)
constexpr std::uint8_t Mode1Register = 0x00;
constexpr std::uint8_t Mode2Register = 0x01;
constexpr std::uint8_t Led0OnLowRegister = 0x06;
constexpr std::uint8_t PrescaleRegister = 0xFE;
constexpr std::uint8_t Mode1Restart = 0x80;
constexpr std::uint8_t Mode1Sleep = 0x10;
constexpr std::uint8_t Mode1AutoIncrement = 0x20;
constexpr std::uint8_t Mode2TotemPole = 0x04;
constexpr unsigned int PwmResolution = 4096;
constexpr double OscillatorFrequencyHz = 25'000'000.0;
#endif

Error make_error(ErrorCode code, std::string message)
{
    return Error{Module::Motor, code, std::move(message)};
}
} // namespace

struct Pca9685Pwm::Impl
{
    explicit Impl(config::PWMConfig configuration) : cfg(std::move(configuration)) {}

    config::PWMConfig cfg;
    int fd{-1};
    bool healthy{false};

#if defined(__linux__)
    Result<void> write_register(std::uint8_t reg, std::uint8_t value)
    {
        const std::uint8_t bytes[]{reg, value};
        ssize_t written{};
        do
        {
            written = ::write(fd, bytes, sizeof(bytes));
        } while (written < 0 && errno == EINTR);
        if (written != static_cast<ssize_t>(sizeof(bytes)))
        {
            return std::unexpected(make_error(
                ErrorCode::IoFailure,
                "Failed to write PCA9685 register: " + std::string(std::strerror(errno))));
        }
        return {};
    }

    Result<std::uint8_t> read_register(std::uint8_t reg)
    {
        ssize_t written{};
        do
        {
            written = ::write(fd, &reg, sizeof(reg));
        } while (written < 0 && errno == EINTR);
        if (written != static_cast<ssize_t>(sizeof(reg)))
        {
            return std::unexpected(make_error(
                ErrorCode::IoFailure,
                "Failed to select PCA9685 register: " + std::string(std::strerror(errno))));
        }
        std::uint8_t value{};
        do
        {
            written = ::read(fd, &value, sizeof(value));
        } while (written < 0 && errno == EINTR);
        if (written != static_cast<ssize_t>(sizeof(value)))
        {
            return std::unexpected(make_error(
                ErrorCode::IoFailure,
                "Failed to read PCA9685 register: " + std::string(std::strerror(errno))));
        }
        return value;
    }

    Result<void> set_channel_duty(double duty)
    {
        const std::uint8_t base_register = static_cast<std::uint8_t>(
            Led0OnLowRegister + (4U * static_cast<unsigned int>(cfg.channel)));
        std::uint8_t bytes[5]{base_register, 0, 0, 0, 0};

        if (duty == 1.0)
        {
            bytes[2] = 0x10;
        }
        else if (duty == 0.0)
        {
            bytes[4] = 0x10;
        }
        else
        {
            const auto off_count = static_cast<std::uint16_t>(
                std::lround(duty * static_cast<double>(PwmResolution)));
            if (off_count >= PwmResolution)
            {
                bytes[2] = 0x10;
            }
            else
            {
                bytes[3] = static_cast<std::uint8_t>(off_count & 0xFFU);
                bytes[4] = static_cast<std::uint8_t>((off_count >> 8U) & 0x0FU);
            }
        }

        ssize_t written{};
        do
        {
            written = ::write(fd, bytes, sizeof(bytes));
        } while (written < 0 && errno == EINTR);
        if (written != static_cast<ssize_t>(sizeof(bytes)))
        {
            return std::unexpected(make_error(
                ErrorCode::IoFailure,
                "Failed to write PCA9685 PWM channel: " + std::string(std::strerror(errno))));
        }
        return {};
    }

    Result<void> set_frequency(int frequency_hz)
    {
        const double prescale_value =
            OscillatorFrequencyHz /
                (static_cast<double>(PwmResolution) * static_cast<double>(frequency_hz)) -
            1.0;
        const auto prescale = static_cast<std::uint8_t>(std::lround(prescale_value));

        auto current_mode = read_register(Mode1Register);
        if (!current_mode)
        {
            return std::unexpected(current_mode.error());
        }
        const std::uint8_t sleep_mode = static_cast<std::uint8_t>(*current_mode | Mode1Sleep);
        if (auto result = write_register(Mode1Register, sleep_mode); !result)
        {
            return result;
        }
        if (auto result = write_register(PrescaleRegister, prescale); !result)
        {
            return result;
        }
        const std::uint8_t wake_mode = static_cast<std::uint8_t>(*current_mode & ~Mode1Sleep);
        if (auto result = write_register(Mode1Register, wake_mode); !result)
        {
            return result;
        }
        std::this_thread::sleep_for(std::chrono::microseconds(500));
        return write_register(Mode1Register, static_cast<std::uint8_t>(wake_mode | Mode1Restart));
    }
#endif

    void close_device() noexcept
    {
#if defined(__linux__)
        if (fd >= 0)
        {
            ::close(fd);
            fd = -1;
        }
#endif
        healthy = false;
    }
};

Pca9685Pwm::Pca9685Pwm(config::PWMConfig config)
    : m_impl(std::make_unique<Impl>(std::move(config)))
{
}

Pca9685Pwm::~Pca9685Pwm()
{
#if defined(__linux__)
    if (m_impl->fd >= 0)
    {
        if (auto result = m_impl->set_channel_duty(0.0); !result)
        {
            std::fprintf(stderr, "Failed to turn off PCA9685 output during destruction: %s\n",
                         result.error().message.c_str());
        }
    }
#endif
    m_impl->close_device();
}

Result<void> Pca9685Pwm::initialize()
{
    if (m_impl->cfg.channel > 15U || m_impl->cfg.address > 0x7FU ||
        m_impl->cfg.frequency_hz < 24 || m_impl->cfg.frequency_hz > 1526 ||
        m_impl->cfg.i2c_device.empty())
    {
        return std::unexpected(make_error(ErrorCode::InvalidArgument, "Invalid PCA9685 configuration"));
    }
    if (m_impl->healthy)
    {
        return {};
    }

#if defined(__linux__)
    m_impl->fd = ::open(m_impl->cfg.i2c_device.c_str(), O_RDWR);
    if (m_impl->fd < 0)
    {
        return std::unexpected(make_error(
            ErrorCode::DeviceNotFound,
            "Unable to open I2C device " + m_impl->cfg.i2c_device + ": " + std::strerror(errno)));
    }
    if (::ioctl(m_impl->fd, I2C_SLAVE, m_impl->cfg.address) < 0)
    {
        const Error error = make_error(
            ErrorCode::DeviceNotFound,
            "Unable to select PCA9685 I2C address: " + std::string(std::strerror(errno)));
        m_impl->close_device();
        return std::unexpected(error);
    }

    if (auto result = m_impl->write_register(Mode1Register, Mode1AutoIncrement); !result)
    {
        const Error error = result.error();
        m_impl->close_device();
        return std::unexpected(error);
    }
    if (auto result = m_impl->set_channel_duty(0.0); !result)
    {
        const Error error = result.error();
        m_impl->close_device();
        return std::unexpected(error);
    }
    if (auto result = m_impl->write_register(Mode2Register, Mode2TotemPole); !result)
    {
        const Error error = result.error();
        m_impl->close_device();
        return std::unexpected(error);
    }
    if (auto result = m_impl->set_frequency(m_impl->cfg.frequency_hz); !result)
    {
        const Error error = result.error();
        m_impl->close_device();
        return std::unexpected(error);
    }

    m_impl->healthy = true;
    return {};
#else
    return std::unexpected(make_error(
        ErrorCode::DeviceNotFound, "PCA9685 I2C access is supported only on Linux"));
#endif
}

Result<void> Pca9685Pwm::set(const PWMCommand& cmd)
{
    if (!m_impl->healthy)
    {
        return std::unexpected(make_error(ErrorCode::NotInitialized, "PCA9685 PWM is not initialized"));
    }
    if (!std::isfinite(cmd.duty) || cmd.duty < 0.0 || cmd.duty > 1.0)
    {
        return std::unexpected(make_error(ErrorCode::InvalidArgument, "PWM duty must be finite and in [0, 1]"));
    }

#if defined(__linux__)
    return m_impl->set_channel_duty(cmd.duty);
#else
    return std::unexpected(make_error(
        ErrorCode::DeviceNotFound, "PCA9685 I2C access is supported only on Linux"));
#endif
}

Result<void> Pca9685Pwm::turnOn()
{
    return set(PWMCommand{.duty = 1.0});
}

Result<void> Pca9685Pwm::turnOff()
{
    return set(PWMCommand{.duty = 0.0});
}

} // namespace robot::drivers
