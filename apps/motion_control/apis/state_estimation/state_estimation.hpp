#pragma once

#include "ModuleType.hpp"
#include "IStateEstimation.hpp"

#include <memory>

#include "hardware/encoders/encoders.hpp"
#include "hardware/motors/motors.hpp"
#include "hardware/imu/imu.hpp"

class StateEstimation: public IStateEstimation, public ModuleType {

    State_t m_state{0.0};
    Encoders& ecdrs;
    Imu& imu;

    public:
        explicit StateEstimation(Encoders& encoders, Imu& imu_in) noexcept;
        ~StateEstimation();        
        void init(void) noexcept override;
		void run(void) noexcept override;
		void deInit(void) noexcept override;
        State_t getState(void) override {return m_state;};
};