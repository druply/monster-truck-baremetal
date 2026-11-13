#pragma once
#include "robot/types/result.hpp"
#include "robot/types/encoder_tick.hpp"

namespace robot::app
{
    class Demo
    {
    public:
       
        Result<bool> update(Tick& right_encoder, Tick& left_encoder);
    };

};