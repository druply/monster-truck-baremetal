#pragma once
#include "robot/hal/i_encoder.hpp"
#include "robot/hal/i_clock.hpp"
#include "robot/types/timestamp.hpp"
#include <numbers>

namespace robot::core
{

    ////// Odometry calibration  /////
    // constexpr float TIRE_RADIUS			 =           0.0698; // ~0.7 meters
    constexpr double TIRE_RADIUS = 0.0705;              // ~0.7 meters
    constexpr double TIRE_DIAMETER = TIRE_RADIUS * 2.0; // meters
    constexpr double PULSES_PER_ROTATION = 15;

    // tire circumfernce = 0.435485 meters
    // constexpr double DISTANCE_PP = ((pi_f * TIRE_DIAMETER) / PULSES_PER_ROTATION); // distance per tick in meters

    constexpr double DISTANCE_PER_TICK =
        (std::numbers::pi_v<double> * TIRE_DIAMETER) / PULSES_PER_ROTATION;

    class Odometry
    {

        // robot::EncoderTick &m_right_encoder;
        // robot::EncoderTick &m_left_encoder;
        robot::Timestamp m_prev_timestamp_encoder_right{0};
        robot::Timestamp m_prev_timestamp_encoder_left{0};
        double m_prev_right_encoder_distance{0};
        double m_prev_left_encoder_distance{0};

        void calculate_encoder_odometry(EncoderTick& right_encoder, EncoderTick& left_encoder);

    public:
        void update(EncoderTick& right_encoder, EncoderTick& left_encoder, ImuSample& imu_sample);
    };
}