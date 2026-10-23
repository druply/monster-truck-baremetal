#pragma once
#include "robot/hal/i_encoder.hpp"
#include "robot/hal/i_clock.hpp"
#include "robot/types/timestamp.hpp"
#include <numbers>

namespace robot::controllers
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

        hal::IEncoder &m_right_encoder;
        hal::IEncoder &m_left_encoder;
        robot::Timestamp m_prev_timestamp_encoder_right{0};
        robot::Timestamp m_prev_timestamp_encoder_left{0};
        double m_prev_right_encoder_distance{0};
        double m_prev_left_encoder_distance{0};

    public:
        Odometry(hal::IEncoder &left_encoder, hal::IEncoder &right_encoder) : m_right_encoder(right_encoder), m_left_encoder(left_encoder) {}
        void step(void);
    };
}