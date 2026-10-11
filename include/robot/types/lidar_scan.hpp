// lidar_scan.hpp
#pragma once
#include <vector>
#include "robot/types/imu_sample.hpp"

namespace robot {
    struct LidarPoint {
        float angle_rad{};
        float range_m{};
        float intensity{};
    };

    struct LidarScan {
        Timestamp stamp{};
        std::vector<LidarPoint> points;
        float angle_min{}, angle_max{};
        float range_min{}, range_max{};
    };
}