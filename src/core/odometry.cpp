#include "robot/core/odometry.hpp"

#include "robot/types/result.hpp"

namespace robot::core
{
namespace
{
Result<void> validate_sample(const EncoderTick& current, const EncoderTick& previous)
{
    if (current.ticks < previous.ticks)
    {
        return std::unexpected(Error{
            Module::Encoder,
            ErrorCode::InvalidArgument,
            "Cumulative encoder ticks moved backwards"});
    }
    if (current.stamp < previous.stamp)
    {
        return std::unexpected(Error{
            Module::Encoder,
            ErrorCode::InvalidArgument,
            "Encoder timestamp moved backwards"});
    }
    return {};
}

double velocity_from_delta(std::uint64_t tick_delta, Timestamp time_delta_ms)
{
    if (time_delta_ms == 0)
    {
        return 0.0;
    }
    const double distance_m = static_cast<double>(tick_delta) * DISTANCE_PER_TICK;
    const double elapsed_seconds = static_cast<double>(time_delta_ms) / 1000.0;
    return distance_m / elapsed_seconds;
}
} // namespace

Result<OdometryState> Odometry::update(
    const EncoderTick& right_encoder,
    const EncoderTick& left_encoder)
{
    if (!m_has_previous_sample)
    {
        m_previous_right = right_encoder;
        m_previous_left = left_encoder;
        m_has_previous_sample = true;
        m_state.right_distance_m = static_cast<double>(right_encoder.ticks) * DISTANCE_PER_TICK;
        m_state.left_distance_m = static_cast<double>(left_encoder.ticks) * DISTANCE_PER_TICK;
        return m_state;
    }

    if (auto result = validate_sample(right_encoder, m_previous_right); !result)
    {
        return std::unexpected(result.error());
    }
    if (auto result = validate_sample(left_encoder, m_previous_left); !result)
    {
        return std::unexpected(result.error());
    }

    const std::uint64_t right_tick_delta = right_encoder.ticks - m_previous_right.ticks;
    const std::uint64_t left_tick_delta = left_encoder.ticks - m_previous_left.ticks;
    const Timestamp right_time_delta = right_encoder.stamp - m_previous_right.stamp;
    const Timestamp left_time_delta = left_encoder.stamp - m_previous_left.stamp;

    m_state.right_distance_m += static_cast<double>(right_tick_delta) * DISTANCE_PER_TICK;
    m_state.left_distance_m += static_cast<double>(left_tick_delta) * DISTANCE_PER_TICK;
    m_state.right_velocity_mps = velocity_from_delta(right_tick_delta, right_time_delta);
    m_state.left_velocity_mps = velocity_from_delta(left_tick_delta, left_time_delta);
    m_previous_right = right_encoder;
    m_previous_left = left_encoder;
    return m_state;
}
} // namespace robot::core
