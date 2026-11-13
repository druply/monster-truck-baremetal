#pragma once

#include <numbers>
#include "robot/types/encoder_tick.hpp"
#include "robot/types/result.hpp"

namespace robot::core
{
    constexpr double TIRE_RADIUS = 0.0705;
    constexpr double PULSES_PER_ROTATION = 15.0;
    constexpr double DISTANCE_PER_TICK =
        (2.0 * std::numbers::pi_v<double> * TIRE_RADIUS) / PULSES_PER_ROTATION;

    struct OdometryState
    {
        double left_distance_m{};
        double right_distance_m{};
        double left_velocity_mps{};
        double right_velocity_mps{};
    };

    class Odometry
    {
        bool m_has_previous_sample{false};
        EncoderTick m_previous_right{};
        EncoderTick m_previous_left{};
        OdometryState m_state{};

    public:
        [[nodiscard]] Result<OdometryState> update(
            const EncoderTick& right_encoder,
            const EncoderTick& left_encoder);
    };
} // namespace robot::core
