#pragma once
#include "ModuleType.hpp"
#include "trajectory_type.hpp"
#include <vector>
#include <utility>

class TrajectoryFollower: public ModuleType {
    std::vector<TrajectoryPoint> _trajectory;
    CarState _state;
    std::pair<float, float> _control_vars;
    public:
        TrajectoryFollower();
        ~TrajectoryFollower();
        void init(void) noexcept override;
		void run(void) noexcept override;
		void deInit(void) noexcept override;
        void setTrajectory(std::vector<TrajectoryPoint> traj);
        void setCarState(CarState state);
        std::pair<float, float> getControlVariables(void);
};