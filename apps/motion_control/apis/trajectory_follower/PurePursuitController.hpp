#pragma once
#include <vector>
#include <cmath>
#include "trajectory_type.hpp"
#include <algorithm> 
#include <iostream>

class PurePursuitController {
private:
    float look_ahead_distance_;
    float wheelbase_; // Distance between front and rear axles
    int last_target_idx{0};

    // Helper to calculate Euclidean distance
    float distance(float x1, float y1, float x2, float y2) {
        return std::hypot(x1 - x2, y1 - y2);
    }

public:
    PurePursuitController(float look_ahead, float wheelbase) 
        : look_ahead_distance_(look_ahead), wheelbase_(wheelbase) {}

    // Finds the closest point on the path that is at the look-ahead distance
    int findTargetIndex(const CarState& car, const std::vector<TrajectoryPoint>& path) {
        for (size_t i = last_target_idx; i < path.size(); ++i) {
            float dist = distance(car.x, car.y, path[i].x, path[i].y);
            if (dist >= look_ahead_distance_) {
                return i; // Found the target point to steer towards
            }
        }
        return path.size() - 1; // Default to last point if path ends
    }

    // Computes control commands: {Steering Angle (rad), Velocity (m/s)}
    std::pair<float, float> computeControls(const CarState& car, const std::vector<TrajectoryPoint>& path) {
        int target_idx = findTargetIndex(car, path);
        last_target_idx = target_idx;
        const auto& target_pt = path[target_idx];
        static float last_steering_angle_{0.0f};
        // 1. Transform target point to the car's local coordinate system
        float dx = target_pt.x - car.x;
        float dy = target_pt.y - car.y;
        std::cout << "dx: " << dx << std::endl;
        std::cout << "dy: " << dy << std::endl;
        // if delta is too small then return last value
        if ((dy>0.09f) || (dy<-0.09f)) {
             return {last_steering_angle_, target_pt.v};
        }
        // Local Y coordinate (positive to the left of the car)
        float local_y = -dx * std::sin(car.theta) + dy * std::cos(car.theta);
        float local_x = dx * std::cos(car.theta) + dy * std::sin(car.theta);
        std::cout << "local_y: " << local_y << std::endl;
        std::cout << "local_x: " << local_x << std::endl;
        // CORRECTED: Flipped signs to make Y-positive point RIGHT instead of LEFT
        //float local_y = dx * std::sin(car.theta) - dy * std::cos(car.theta);

        // 2. Pure Pursuit Steering Formula: delta = atan2(2 * L * sin(alpha) / L_f)
        // Which simplifies using local coordinates to:
        // float steering_angle = std::atan2(2.0 * wheelbase_ * local_y, 
        //                                    look_ahead_distance_ * look_ahead_distance_);

        float alpha = std::atan2(local_y,local_x);
        std::cout << "alpha: " << alpha << std::endl;
        float steering_angle = std::atan2(2*wheelbase_*std::sin(alpha), look_ahead_distance_);
        std::cout << "raw steering_angle: " << steering_angle << std::endl;
        // 3. Longitudinal control (simple velocity pass-through or error-based)
        float target_velocity = target_pt.v;
        
        // 4. Rate-Limit Steering (Slew Rate) to enforce physical limits
        float max_change = 0.05f; // Maximum radians the wheel can turn per frame
        float clamped_steering = std::clamp(steering_angle, 
                                        last_steering_angle_ - max_change, 
                                        last_steering_angle_ + max_change);

        // 5. Exponential Moving Average Smoothing
        float smoothed_steering = (0.15f * clamped_steering) + (0.85f * last_steering_angle_);
        last_steering_angle_ = smoothed_steering;

        return {smoothed_steering, target_velocity};
    }
};
