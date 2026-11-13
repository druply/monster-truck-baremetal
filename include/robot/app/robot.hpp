#pragma once
#include <memory>
#include <thread>
#include "robot/hal/i_encoder.hpp"
#include "robot/hal/i_clock.hpp"
// #include "robot/drivers/lm393_encoder.hpp"
#include "mocks/mock_encoder.hpp"
#include "robot/core/odometry.hpp"
#include "robot/types/result.hpp"
#include "utils/clock.hpp"
#include "robot/app/hardware.hpp"
#include "robot/app/demo.hpp"

namespace robot::app
{
    class Robot
    {
        Hardware m_hw;
        std::jthread m_encoder_thread;
        void encoder_loop(std::stop_token stoken);
        robot::core::Odometry m_odometry;
        robot::app::Demo m_demo;
    public:
        Robot(const std::string & /*cfg*/, Hardware &&hw) : m_hw(std::move(hw)) {}
        Result<void> initialize();
        Result<bool> spin_once();
    };

};