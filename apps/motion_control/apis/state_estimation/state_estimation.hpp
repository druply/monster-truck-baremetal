#pragma once

#include "ModuleType.hpp"
#include <memory>

// State
struct State_t {
    float x = 0.0;    // global X (m)
    float y = 0.0;    // global Y (m)
    float vx = 0.0;   // global Vx (m/s)
    float vy = 0.0;   // global Vy (m/s)
    float heading;
    float distance;
};


class StateEstimation: public ModuleType {

    State_t m_state{0.0};
  
    public:
        explicit StateEstimation() noexcept;
        ~StateEstimation();        
        void init(void) noexcept override;
		void run(void) noexcept override;
		void deInit(void) noexcept override;
        State_t getState(void) {return m_state;};
        
};