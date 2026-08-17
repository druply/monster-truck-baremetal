#pragma once
#include <optional>
#include "imu_types.hpp"


class IImu {
public:
    virtual ~IImu() = default;
    virtual std::optional<AllAxes> read_all() = 0;
};