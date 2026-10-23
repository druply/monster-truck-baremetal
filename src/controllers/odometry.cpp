#include "robot/controllers/odometry.hpp"
#include <print>
namespace robot::controllers
{

    void Odometry::step(void)
    {
        double right_encoder_distance = 0;
        double right_encoder_delta_distance = 0;
        [[maybe_unused]] double right_encoder_velocity;
        robot::Timestamp delta_time_encoder_right_local = 0;

        double left_encoder_distance = 0;
        double left_encoder_delta_distance = 0;
        [[maybe_unused]] double left_encoder_velocity;
        robot::Timestamp delta_time_encoder_left_local = 0;

        robot::Result<robot::EncoderTick> right_tick = m_right_encoder.read();
        robot::Result<robot::EncoderTick> left_tick = m_left_encoder.read();

        if (right_tick)
        {
            // calculate right delta
            delta_time_encoder_right_local = right_tick->stamp - m_prev_timestamp_encoder_right;
            m_prev_timestamp_encoder_right = right_tick->stamp;

            // calculate right velocity
            right_encoder_distance = static_cast<double>(right_tick->ticks) * DISTANCE_PER_TICK;

            right_encoder_delta_distance = right_encoder_distance - m_prev_right_encoder_distance;
            double delta_time_encoder_right_seconds = static_cast<double>(delta_time_encoder_right_local) / 1000.0;
            right_encoder_velocity = (right_encoder_delta_distance) / delta_time_encoder_right_seconds;
            // save previous distance
            m_prev_right_encoder_distance = right_encoder_distance;
        }
        if (left_tick)
        {
            // calculate left delta
            delta_time_encoder_left_local = left_tick->stamp - m_prev_timestamp_encoder_left;
            m_prev_timestamp_encoder_left = left_tick->stamp;

            // calculate left velocity
            left_encoder_distance = static_cast<double>(left_tick->ticks) * DISTANCE_PER_TICK;
            
            left_encoder_delta_distance = left_encoder_distance - m_prev_left_encoder_distance;
            double delta_time_encoder_left_seconds = static_cast<double>(delta_time_encoder_left_local) / 1000.0;
            left_encoder_velocity = (left_encoder_delta_distance) / delta_time_encoder_left_seconds;
            // save previous distance
            m_prev_left_encoder_distance = left_encoder_distance;


        }
    }

}