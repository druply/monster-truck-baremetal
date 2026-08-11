#include "trajectory_follower.hpp"
#include "PurePursuitController.hpp"
#include "apis/state_estimation/state_estimation.hpp"


// Initialize controller: Look-ahead = 0.2m, Wheelbase = 0.35m (typical 1/10 RC scale)
static PurePursuitController controller{0.2, 0.35};

TrajectoryFollower::TrajectoryFollower() {}

TrajectoryFollower::~TrajectoryFollower() {}

void TrajectoryFollower::init(void) noexcept {

}

void TrajectoryFollower::run(void) noexcept {
    _control_vars = controller.computeControls(_state, _trajectory);
}

void TrajectoryFollower::deInit(void) noexcept {}

void TrajectoryFollower::setTrajectory(std::vector<TrajectoryPoint> traj) {
    _trajectory = traj;
}

void TrajectoryFollower::setCarState(CarState state) {
    _state = state;
}

std::pair<float, float> TrajectoryFollower::getControlVariables(void) {
    return _control_vars;
}