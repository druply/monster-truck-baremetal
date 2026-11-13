// test/mocks/mock_imu.hpp
#include "robot/hal/i_imu.hpp"

class MockImu : public robot::hal::IImu {
public:
    robot::Result<void> initialize() override { return {}; }
    robot::Result<robot::ImuSample> read() override { 
        //return std::unexpected(robot::Error{robot::Module::Imu, robot::ErrorCode::IoFailure, "Failed to get IMU data"});
        return next_; 
    }
    bool is_healthy() const noexcept override { return healthy_; }

    robot::ImuSample next_{
        0,
        {0.5, 0.3, 0.7},
        {0.1, 0.2, 0.21},
        {0.0, 0.0, 0.0},
        0.0};
    bool healthy_{true};
};