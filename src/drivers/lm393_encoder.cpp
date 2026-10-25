// src/drivers/lm393_encoder.cpp
#include "robot/drivers/lm393_encoder.hpp"

#include <gpiod.h>
#include <poll.h>
#include <unistd.h>
#include <atomic>

namespace robot::drivers
{
    struct Lm393Encoder::Impl
    {
        robot::config::EncoderConfig cfg{};
        std::int64_t ticks{0};
        bool healthy{false};
        std::atomic<bool> reverse{false};
    };

    Lm393Encoder::Lm393Encoder(robot::config::EncoderConfig cfg)
        : cfg_(std::move(cfg)), impl_(std::make_unique<Impl>())
    {
        impl_->cfg = cfg_;
    }

    Lm393Encoder::~Lm393Encoder() = default;

    Result<void> Lm393Encoder::initialize()
    {
        impl_->healthy = true;
        return {};
    }

    Result<void> Lm393Encoder::run()
    {
        return {};
    }

    Result<EncoderTick> Lm393Encoder::read()
    {
        if (!impl_) {
            return std::unexpected(robot::Error{
                robot::Module::Encoder,
                robot::ErrorCode::NotInitialized,
                "LM393 encoder is not initialized"
            });
        }

        const auto next_ticks = ++impl_->ticks;
        return EncoderTick{.stamp = 0, .ticks = next_ticks};
    }

    bool Lm393Encoder::is_healthy() const noexcept
    {
        return impl_ && impl_->healthy;
    }

    void Lm393Encoder::set_direction(bool reverse) noexcept
    {
        if (impl_) {
            impl_->reverse.store(reverse, std::memory_order_relaxed);
        }
    }

    std::int64_t Lm393Encoder::ticks() const noexcept
    {
        return impl_ ? impl_->ticks : 0;
    }

} // namespace robot::drivers