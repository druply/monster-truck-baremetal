#include "robot/hal/i_encoder.hpp"
#include "robot/types/result.hpp"


    class MockEncoder: public robot::hal::IEncoder {
         robot::EncoderTick next_;
                     robot::EncoderTick last_{};
            bool healthy_{true};
        public:
         ~MockEncoder() = default;
         robot::Result<void> initialize() override { return {}; }
         robot::Result<robot::EncoderTick> read() override { 
            //next_.stamp = std::chrono::system_clock::now();
            next_.ticks++;
            last_ = next_;
            return next_; 
        }
        robot::Result<void> run() override {

            return {};
        }
         bool is_healthy() const noexcept override { return healthy_; }


    };
