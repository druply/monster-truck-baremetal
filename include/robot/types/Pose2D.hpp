#pragma once
#include "robot/types/timestamp.hpp"

namespace robot {
    
    struct Pose2D {
        Timestamp stamp{};
        double x{0};
        double y{0};
        double theta{0};
    };
}