#include "robot/controllers/odometry.hpp"
#include <print>
namespace robot::controllers {

        void Odometry::step(void) {
       robot::Result<robot::EncoderTick> right_tick = m_right_encoder.read();
       robot::Result<robot::EncoderTick> left_tick = m_left_encoder.read();
       if (right_tick && left_tick) {
           // Process the encoder ticks to update odometry
           // This is where you would implement the logic to calculate the robot's position and orientation based on the encoder readings.
        std::printf("Right ticks: %lld, Left ticks: %lld\n", right_tick->ticks, left_tick->ticks);
        }
    }

}