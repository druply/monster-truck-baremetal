#include "encoders.hpp"
#include <iostream>
#include <gpiod.hpp>
#include <boost/outcome.hpp>
#include <system_error>

const std::string chip_name = "gpiochip0";

// Type alias for C++23 style error handling
// using HardwareResult = std::expected<gpiod::line, std::error_code>;
namespace outcome = BOOST_OUTCOME_V2_NAMESPACE;
using HardwareResult = outcome::result<gpiod::line, std::error_code>;

Encoders::Encoders() : noexcept m_impl(std::make_unique<Impl>()) {}
Encoders::~Encoders() {}

// Helper function using C++23 std::expected instead of throwing exceptions
HardwareResult setup_gpio_line(unsigned int pin) noexcept
{

    if (pin >= 32)
    {
        return outcome::failure(std::make_error_code(std::errc::invalid_argument));
    }

    // Wrap gpiod calls that may throw into a try/catch and convert exceptions to error_code.
    try
    {
        gpiod::chip chip{chip_name};
        gpiod::line line = chip.get_line(pin);
        line.request({"encoder_right", gpiod::line_request::EVENT_RISING_EDGE, 0});
        return outcome::success(std::move(line));
    }
    catch (const std::system_error &e)
    {
        return outcome::failure(e.code());
    }
    catch (const std::exception &e)
    {
        // map generic exception -> appropriate error_code (adjust mapping as needed)
        return outcome::failure(std::make_error_code(std::errc::io_error));
    }
    catch (...)
    {
        return outcome::failure(std::make_error_code(std::errc::io_error));
    }
}

void Encoders::monitor_encoder_right_thread(void) noexcept
{

    // Initialize hardware safely using C++23 features
    auto result = setup_gpio_line(m_impl->right_encoder_gpio);

    if (!result.has_value())
    {
        // Handle hardware initialization failure safely without crashing
        right_encoder_fault.store(true, std::memory_order_relaxed);
        return;
    }

    gpiod::line line = std::move(result.value());
    const auto timeout = std::chrono::milliseconds(10);

    // Optimized C++23 Lock-free real-time loop
    while (m_impl->m_running.load(std::memory_order_relaxed))
    {
        if (line.event_wait(timeout))
        {
            line.event_read();

            // Atomically increment with relaxed memory order for peak ARM performance
            m_right_encoder.fetch_add(1, std::memory_order_relaxed);
        }
    }

    line.release();
}

int64_t Encoders::get_left_pulses() const noexcept { return m_impl->m_left_encoder.load(std::memory_order_acquire); }
int64_t Encoders::get_right_pulses() const noexcept { return m_impl->m_right_encoder.load(std::memory_order_acquire); }

void Encoders::monitor_encoder_left_thread(void) noexcept
{

    // Initialize hardware safely using C++23 features
    auto result = setup_gpio_line(m_impl->left_encoder_gpio);

    if (!result.has_value())
    {
        // Handle hardware initialization failure safely without crashing
        m_left_encoder_fault.store(true, std::memory_order_relaxed);
        return;
    }

    gpiod::line line = std::move(result.value());
    const auto timeout = std::chrono::milliseconds(10);

    // Optimized C++23 Lock-free real-time loop
    while (m_running.load(std::memory_order_relaxed))
    {
        if (line.event_wait(timeout))
        {
            line.event_read();

            // Atomically increment with relaxed memory order for peak ARM performance
            m_left_encoder.fetch_add(1, std::memory_order_relaxed);
        }
    }

    line.release();
}

void Encoders::init(void) noexcept
{
    std::cout << "raspberry pi on" << std::endl;
    // 1. Explicitly set the atomic flag using relaxed ordering (no other threads exist yet)
    m_running.store(true, std::memory_order_relaxed);

    // 2. Spawn the thread only after the flag is guaranteed to be true
    right_worker_thread = std::thread(&Encoders::monitor_encoder_right_thread, this);
    left_worker_thread = std::thread(&Encoders::monitor_encoder_left_thread, this);
}

void Encoders::run(void) noexcept
{
}

void Encoders::deInit(void) noexcept
{
    // Explicitly requesting a stop signals the stop_token and joins the thread.
    // This blocks for a maximum of 10ms (our line.event_wait timeout) and safely exits.
    // m_impl.worker_thread.request_stop();
    m_running.store(false, std::memory_order_release);
    // join thread so we wait for it to finish
    right_worker_thread.join();
    left_worker_thread.join();
}