#pragma once
#include <cstdint>

class IEncoders {
public:
    virtual ~IEncoders() = default;
    virtual int64_t get_left_pulses() const noexcept = 0;
    virtual int64_t get_right_pulses() const noexcept = 0;
};
