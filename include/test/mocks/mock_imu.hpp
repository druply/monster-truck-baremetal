// test/mocks/mock_imu.hpp
#include "robot/hal/i_imu.hpp"

class MockImu : public robot::hal::IImu {
public:
    robot::Result<void> initialize() override { return {}; }
    robot::Result<robot::ImuSample> read() override { return next_; }
    bool is_healthy() const noexcept override { return healthy_; }

    robot::ImuSample next_{};
    bool healthy_{true};
};