// i_imu.hpp
#pragma once
#include "robot/types/imu_sample.hpp"
#include "robot/types/result.hpp"

namespace robot::hal {
    class IImu {
    public:
        virtual ~IImu() = default;
        virtual Result<void> initialize() = 0;
        virtual Result<ImuSample> read() = 0;   // blocking or cached
        [[nodiscard]] virtual bool is_healthy() const noexcept = 0;
    };
}