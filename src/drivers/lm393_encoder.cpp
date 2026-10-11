// src/drivers/lm393_encoder.cpp
#include "robot/drivers/lm393_encoder.hpp"

#include <gpiod.h>
#include <poll.h>
#include <unistd.h>
#include <atomic>

namespace robot::drivers {

struct Lm393Encoder::Impl {
    Lm393EncoderConfig cfg;

    gpiod_chip* chip{nullptr};
    gpiod_line_request* request{nullptr};

    std::int64_t ticks{0};
    Timestamp last_tick_stamp{};
    bool healthy{false};

    std::atomic<bool> reverse{false};

    Result<void> open_and_request() {
        chip = gpiod_chip_open(cfg.gpio_chip.c_str());
        if (!chip) return std::unexpected(Error::DeviceNotFound);

        gpiod_line_settings* settings = gpiod_line_settings_new();
        if (!settings) return std::unexpected(Error::IoFailure);

        gpiod_line_settings_set_direction(settings, GPIOD_LINE_DIRECTION_INPUT);

        // Only falling edges are needed: one pulse per hole.
        // Rising edges would double-count.
        gpiod_line_settings_set_edge_detection(settings, GPIOD_LINE_EDGE_FALLING);

        if (cfg.debounce_ns > 0) {
            gpiod_line_settings_set_debounce_period_us(
                settings, cfg.debounce_ns / 1000);
        }
        if (cfg.use_bias_pull_up) {
            gpiod_line_settings_set_bias(settings, GPIOD_LINE_BIAS_PULL_UP);
        }

        gpiod_line_config* line_cfg = gpiod_line_config_new();
        if (!line_cfg) {
            gpiod_line_settings_free(settings);
            return std::unexpected(Error::IoFailure);
        }

        unsigned int offsets[1] = {cfg.line};
        if (gpiod_line_config_add_line_settings(line_cfg, offsets, 1, settings) < 0) {
            gpiod_line_config_free(line_cfg);
            gpiod_line_settings_free(settings);
            return std::unexpected(Error::InvalidArgument);
        }

        gpiod_request_config* req_cfg = gpiod_request_config_new();
        if (!req_cfg) {
            gpiod_line_config_free(line_cfg);
            gpiod_line_settings_free(settings);
            return std::unexpected(Error::IoFailure);
        }
        gpiod_request_config_set_consumer(req_cfg, "robot_lm393");

        request = gpiod_chip_request_lines(chip, req_cfg, line_cfg);

        gpiod_request_config_free(req_cfg);
        gpiod_line_config_free(line_cfg);
        gpiod_line_settings_free(settings);

        if (!request) return std::unexpected(Error::IoFailure);
        return {};
    }

    bool wait_for_event(int timeout_ms) {
        int fd = gpiod_line_request_get_fd(request);
        struct pollfd pfd{};
        pfd.fd = fd;
        pfd.events = POLLIN;
        int rc = ::poll(&pfd, 1, timeout_ms);
        return rc > 0 && (pfd.revents & POLLIN);
    }

    void drain_events() {
        gpiod_edge_event_buffer* buf = gpiod_edge_event_buffer_new(32);
        if (!buf) {
            healthy = false;
            return;
        }

        const bool rev = reverse.load(std::memory_order_relaxed);

        while (true) {
            int n = gpiod_line_request_read_edge_events(request, buf, 32);
            if (n <= 0) break;

            for (int i = 0; i < n; ++i) {
                gpiod_edge_event* ev = gpiod_edge_event_buffer_get_event(buf, i);
                if (!ev) continue;

                std::uint64_t ns = gpiod_edge_event_get_timestamp_ns(ev);

                // One falling edge = one hole = one tick.
                // Sign comes from the motor direction hint.
                std::int64_t delta = rev ? -1 : +1;
                if (cfg.invert) delta = -delta;

                ticks += delta;
                last_tick_stamp = Timestamp{std::chrono::nanoseconds{ns}};
            }
        }

        gpiod_edge_event_buffer_free(buf);
    }

    ~Impl() {
        if (request) gpiod_line_request_release(request);
        if (chip) gpiod_chip_close(chip);
    }
};

Lm393Encoder::Lm393Encoder(Lm393EncoderConfig cfg)
    : impl_(std::make_unique<Impl>(Impl{std::move(cfg)})) {}

Lm393Encoder::~Lm393Encoder() = default;

Result<void> Lm393Encoder::initialize() {
    if (auto r = impl_->open_and_request(); !r) return r;
    impl_->healthy = true;
    return {};
}

Result<EncoderTick> Lm393Encoder::read() {
    if (!impl_->healthy) return std::unexpected(Error::NotInitialized);

    if (impl_->wait_for_event(/*timeout_ms=*/1)) {
        impl_->drain_events();
    }

    if (!impl_->healthy) return std::unexpected(Error::IoFailure);

    EncoderTick tick;
    tick.ticks = impl_->ticks;
    tick.stamp = impl_->last_tick_stamp.time_since_epoch().count() == 0
                     ? Timestamp{}
                     : impl_->last_tick_stamp;
    return tick;
}

bool Lm393Encoder::is_healthy() const noexcept {
    return impl_->healthy;
}

void Lm393Encoder::set_direction(bool reverse) noexcept {
    impl_->reverse.store(reverse, std::memory_order_relaxed);
}

std::int64_t Lm393Encoder::ticks() const noexcept {
    return impl_->ticks;
}

} // namespace robot::drivers