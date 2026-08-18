#pragma once
#include "imu_types.hpp"
#include <cstddef> 
#include <cassert>

class LowPassFilter3D {
private:
    Vector3 filtered_value;
    float alpha;

public:
    // Added static assert check capability for runtime/compile-time safety
    explicit constexpr LowPassFilter3D(float smoothing_factor) noexcept
        : filtered_value{0.0f, 0.0f, 0.0f},
          alpha(smoothing_factor) {
              // Standard assert works in constexpr contexts in Modern C++
              assert(smoothing_factor > 0.0f && smoothing_factor <= 1.0f);
          }

    // Changed to pass-by-value to utilize CPU registers instead of pointers
    constexpr void reset(Vector3 initial_raw) noexcept {
        filtered_value = initial_raw;
    }

    // Pass-by-value for 'raw', returns const ref to eliminate return copy overhead
    [[nodiscard]] constexpr const Vector3& update(Vector3 raw) noexcept {
        constexpr std::size_t AXES_COUNT = 3;
        
        // Loop is clean and easily vectorized by modern compilers (O3 / AVX)
        for (std::size_t i = 0; i < AXES_COUNT; ++i) {
            filtered_value[i] += alpha * (raw[i] - filtered_value[i]);
        }
        return filtered_value;
    }

    // Added getter for zero-overhead state inspection without mutating
    [[nodiscard]] constexpr const Vector3& get() const noexcept {
        return filtered_value;
    }
};
