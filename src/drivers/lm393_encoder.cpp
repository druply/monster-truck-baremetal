// src/drivers/lm393_encoder.cpp
#include "robot/drivers/lm393_encoder.hpp"

#include <gpiod.h>
#include <poll.h>
#include <unistd.h>
#include <atomic>

namespace robot::drivers
{

    using HardwareResult = std::expected<gpiod::line, std::error_code>;

    struct Lm393Encoder::Impl
    {
        Lm393EncoderConfig cfg;

        // gpiod_chip *chip{nullptr};
        // gpiod_line_request *request{nullptr};

        std::int64_t ticks{0};
        // Timestamp last_tick_stamp{};
        bool healthy{false};

        std::atomic<bool> reverse{false};

        HardwareResult setup_gpio_line(void) noexcept
        {

            if (cfg.line >= 32)
            {
                return std::unexpected(std::make_error_code(std::errc::invalid_argument));
            }

            // Wrap gpiod calls that may throw into a try/catch and convert exceptions to error_code.
            try
            {
                gpiod::chip chip{cfg.gpio_chip};
                gpiod::line line = chip.get_line(cfg.line);
                line.request({cfg.line_name, gpiod::line_request::EVENT_RISING_EDGE, 0});
                return std::unexpected(std::move(line));
            }
            catch (const std::system_error &e)
            {
                return std::unexpected(e.code());
            }
            catch (const std::exception &e)
            {
                // map generic exception -> appropriate error_code (adjust mapping as needed)
                return std::unexpected(std::make_error_code(std::errc::io_error));
            }
            catch (...)
            {
                return std::unexpected(std::make_error_code(std::errc::io_error));
            }
        }

        ~Impl()
        {
            // if (request)
            //     gpiod_line_request_release(request);
            // if (chip)
            //     gpiod_chip_close(chip);
        }
    };

    Lm393Encoder::Lm393Encoder(Lm393EncoderConfig cfg)
        : impl_(std::make_unique<Impl>(Impl{std::move(cfg)})) {}

    Lm393Encoder::~Lm393Encoder() = default;

    Result<void> Lm393Encoder::initialize()
    {
        HardwareResult result = impl_->setup_gpio_line();

        if (!result.has_value())
        {
            // Handle hardware initialization failure safely without crashing
            impl_->healthy = false;
            return;
        }

        impl_->healthy = true;
        return {};
    }

    Result<void> Lm393Encoder::run()
    {
        while (m_impl->m_running.load(std::memory_order_relaxed))
        {
            if (line.event_wait(timeout))
            {
                line.event_read();
                // Atomically increment with relaxed memory order for peak ARM performance
                m_right_encoder.fetch_add(1, std::memory_order_relaxed);
            }
        }
        Í
    }

    Result<EncoderTick> Lm393Encoder::read()
    {
        EncoderTick tick;
        return tick;
    }

    bool Lm393Encoder::is_healthy() const noexcept
    {
        return impl_->healthy;
    }

    void Lm393Encoder::set_direction(bool reverse) noexcept
    {
        impl_->reverse.store(reverse, std::memory_order_relaxed);
    }

    std::int64_t Lm393Encoder::ticks() const noexcept
    {
        return impl_->ticks;
    }

} // namespace robot::drivers