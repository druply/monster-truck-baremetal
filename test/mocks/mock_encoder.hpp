#pragma once

#include "robot/hal/i_encoder.hpp"

#include <chrono>
#include <cstdint>

class MockEncoder final : public robot::hal::IEncoder
{
    robot::EncoderTick m_encoder{};
    bool m_healthy{false};

public:
    robot::Result<void> initialize() override
    {
        m_healthy = true;
        m_encoder = {};
        return {};
    }

    [[nodiscard]] robot::Result<robot::EncoderTick> read() const override
    {
        if (!m_healthy)
        {
            return std::unexpected(robot::Error{
                robot::Module::Encoder,
                robot::ErrorCode::NotInitialized,
                "Mock encoder is not initialized"});
        }
        return m_encoder;
    }

    robot::Result<void> run() override
    {
        if (!m_healthy)
        {
            return std::unexpected(robot::Error{
                robot::Module::Encoder,
                robot::ErrorCode::NotInitialized,
                "Mock encoder is not initialized"});
        }
        ++m_encoder.ticks;
        m_encoder.stamp = static_cast<robot::Timestamp>(
            std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now().time_since_epoch())
                .count());
        return {};
    }

    [[nodiscard]] bool is_healthy() const noexcept override { return m_healthy; }
};
