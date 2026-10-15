#pragma once
#include <memory>
#include "robot/hal/i_encoder.hpp"
#include "robot/hal/i_clock.hpp"
//#include "robot/drivers/lm393_encoder.hpp"
#include "mocks/mock_encoder.hpp"
#include "robot/controllers/odometry.hpp"
#include "robot/types/result.hpp"
#include "utils/clock.hpp"

namespace robot
{
    class Robot
    {
        std::unique_ptr<hal::IEncoder> m_right_encoder;
        std::unique_ptr<hal::IEncoder> m_left_encoder;
        std::unique_ptr<controllers::Odometry> m_odometry;
        std::unique_ptr<hal::IClock> m_clock; 

    public:
        Robot() : m_right_encoder(std::make_unique<MockEncoder>()), 
                  m_left_encoder(std::make_unique<MockEncoder>()), 
                  m_odometry(std::make_unique<controllers::Odometry>(*m_left_encoder, *m_right_encoder)), 
                  m_clock(std::make_unique<utils::Clock>()) {}
        Result<void> initialize();
    };

};