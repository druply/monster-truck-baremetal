#pragma once

// Represents a point in the planned trajectory
struct TrajectoryPoint {
    float x;
    float y;
    float v; // Target velocity at this point
};

// Represents the current state of the RC Car
struct CarState {
    float x;
    float y;
    float theta; // Heading in radians
};
