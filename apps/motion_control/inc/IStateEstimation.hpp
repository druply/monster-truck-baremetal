#pragma once
#include "state_types.hpp"

class IStateEstimation {
    
    public:
        virtual ~IStateEstimation() = default;
        virtual State_t getState(void) = 0;
};