// test/mocks/mock_motor.hpp
#include "robot/hal/i_motor.hpp"

class MockMotor : public robot::hal::IMotor {
public:
    robot::Result<void> initialize() override { return {}; }
    robot::Result<void> set(const robot::MotorCommand& cmd) override {
        last_ = cmd;
        return {};
    }
    robot::Result<void> stop() override { last_ = {}; return {}; }
    bool is_healthy() const noexcept override { return healthy_; }

    robot::MotorCommand last_{};
    bool healthy_{true};
};