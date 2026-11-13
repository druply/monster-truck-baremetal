// src/drivers/lm393_encoder.cpp
#include "robot/drivers/lm393_encoder.hpp"

#include <gpiod.h>
#include <poll.h>
#include <unistd.h>
#include <atomic>
#include <chrono>

namespace robot::drivers
{
    using HardwareResult = std::expected<gpiod::line, std::string>;

    HardwareResult setup_gpio_line(robot::config::EncoderConfig &cfg) noexcept
    {

        if (cfg.line >= 32)
        {
            return std::unexpected("Invalid GPIO pin number: " + std::to_string(cfg.line));
        }

        // Wrap gpiod calls that may throw into a try/catch and convert exceptions to error_code.
        try
        {
            gpiod::chip chip{cfg.gpio_chip};
            gpiod::line line = chip.get_line(cfg.line);
            line.request({cfg.line_name, gpiod::line_request::EVENT_RISING_EDGE, 0});
            return HardwareResult::success(std::move(line));
        }
        catch (const std::system_error &e)
        {
            return std::unexpected("System error occurred: " + std::string(e.what()));
        }
        catch (const std::exception &e)
        {
            // map generic exception -> appropriate error_code (adjust mapping as needed)
            return std::unexpected("Unexpected error occurred: " + std::string(e.what()));
        }
        catch (...)
        {
            return std::unexpected("Unknown error occurred");
        }
    }

    struct Lm393Encoder::Impl
    {
        robot::config::EncoderConfig cfg{};
        std::atomic<Tick> ticks{0};
        std::atomic<EncoderTick> last_tick{};
        bool healthy{false};
        gpiod::line line;
        std::atomic<bool> reverse{false};
        const auto timeout = std::chrono::milliseconds(10);
    };

    Lm393Encoder::Lm393Encoder(robot::config::EncoderConfig cfg)
        : cfg_(std::move(cfg)), impl_(std::make_unique<Impl>())
    {
        impl_->cfg = cfg_;
    }

    Lm393Encoder::~Lm393Encoder() = default;

    Result<void> Lm393Encoder::initialize()
    {
        // Initialize hardware safely using C++23 features
        HardwareResult result = setup_gpio_line(impl_->cfg);

        if (!result)
        {
            return std::unexpected(Error{
                Module::Encoder,
                ErrorCode::IoFailure,
                "Failed to initialize GPIO line: " + result.error()});
        }

        impl_->line = std::move(result.value()); // move ownership of the line into impl_
        impl_->healthy = true;
        return {};
    }

    std::optional<EncoderTick> Lm393Encoder::read()
    {

        if (!is_healthy())
        {
            return std::nullopt;
        }
        return impl_->last_tick.load(std::memory_order_relaxed);
        // if (impl_->line.event_wait(impl_->timeout))
        // {
        //     auto event = impl_->line.event_read();
        //     // Atomically increment with relaxed memory order for peak ARM performance
        //     impl_->ticks.fetch_add(1, std::memory_order_relaxed);

        //     const auto event_time = std::chrono::seconds{event.ts.tv_sec} + std::chrono::nanoseconds{event.ts.tv_nsec};
        //     const auto stamp_ms =
        //         std::chrono::duration_cast<std::chrono::milliseconds>(event_time).count();

        //     return EncoderTick{
        //         .stamp = static_cast<Timestamp>(stamp_ms),
        //         .ticks = impl_->ticks.load(std::memory_order_relaxed)};
        // }
        // return std::nullopt;
    }

    Result<void> Lm393Encoder::run()
    {
        if (!is_healthy())
        {
            return std::unexpected(Error{
                Module::Encoder,
                ErrorCode::IoFailure,
                "Encoder is not healthy"});
        }
        if (impl_->line.event_wait(impl_->timeout))
        {
            auto event = impl_->line.event_read();
            // Atomically increment with relaxed memory order for peak ARM performance
            impl_->ticks.fetch_add(1, std::memory_order_relaxed);

            const auto event_time = std::chrono::seconds{event.ts.tv_sec} + std::chrono::nanoseconds{event.ts.tv_nsec};
            const auto stamp_ms =
                std::chrono::duration_cast<std::chrono::milliseconds>(event_time).count();
            impl_->last_tick.store(EncoderTick{
                .stamp = static_cast<Timestamp>(stamp_ms),
                .ticks = impl_->ticks.load(std::memory_order_relaxed)
            }, std::memory_order_relaxed);
            // return EncoderTick{
            //     .stamp = static_cast<Timestamp>(stamp_ms),
            //     .ticks = impl_->ticks.load(std::memory_order_relaxed)};
        }
        return Result<void>{};
    }

    bool Lm393Encoder::is_healthy() const noexcept
    {
        return impl_ && impl_->healthy;
    }

    // void Lm393Encoder::set_direction(bool reverse) noexcept
    // {
    //     if (impl_)
    //     {
    //         impl_->reverse.store(reverse, std::memory_order_relaxed);
    //     }
    // }

    Tick Lm393Encoder::ticks() const noexcept
    {
        return impl_ ? impl_->ticks.load(std::memory_order_relaxed) : 0;
    }

} // namespace robot::drivers