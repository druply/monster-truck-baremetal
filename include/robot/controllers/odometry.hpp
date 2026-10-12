#pragma once
#include "robot/hal/i_encoder.hpp"

namespace robot::controllers {

    class Odometry {

        hal::IEncoder& m_right_encoder;
        hal::IEncoder& m_left_encoder;

        public:
        Odometry(hal::IEncoder& left_encoder, hal::IEncoder& right_encoder) : m_left_encoder(left_encoder), m_right_encoder(right_encoder) {}
        void step(void);
        
    };
}