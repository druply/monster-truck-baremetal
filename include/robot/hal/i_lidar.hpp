// i_lidar.hpp
#pragma once
#include <functional>
#include "robot/types/lidar_scan.hpp"
#include "robot/types/result.hpp"

namespace robot::hal {
    class ILidar {
    public:
        using ScanCallback = std::function<void(LidarScan&&)>;

        virtual ~ILidar() = default;
        virtual Result<void> initialize() = 0;
        virtual Result<void> start(ScanCallback cb) = 0;  // async stream
        virtual Result<void> stop() = 0;
        [[nodiscard]] virtual bool is_healthy() const noexcept = 0;
    };
}