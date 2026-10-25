#pragma once
#include <memory>
#include "robot/hal/i_encoder.hpp"
#include "robot/hal/i_clock.hpp"
//#include "robot/drivers/lm393_encoder.hpp"
#include "mocks/mock_encoder.hpp"
#include "robot/controllers/odometry.hpp"
#include "robot/types/result.hpp"
#include "utils/clock.hpp"
#include "robot/app/hardware.hpp"

namespace robot::app
{
    class Robot
    {

    Hardware m_hw;

    public:
        Robot(const std::string& /*cfg*/, Hardware&& hw) : m_hw(std::move(hw)) {}
        Result<void> initialize();
    };

};