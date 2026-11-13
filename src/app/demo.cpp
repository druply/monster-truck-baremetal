#include "robot/app/demo.hpp"
#include <print>

namespace robot::app
{

        Result<bool> Demo::update(Tick& right_encoder, Tick& left_encoder)
        {
            std::print("Demo::update() called with right_encoder: {}, left_encoder: {}\n", right_encoder, left_encoder);
            return Result<bool>{true};
        }

};