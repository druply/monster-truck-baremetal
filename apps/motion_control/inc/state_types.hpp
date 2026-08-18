#pragma once

// State
struct State_t {
    float x = 0.0;    // global X (m)
    float y = 0.0;    // global Y (m)
    float vx = 0.0;   // global Vx (m/s)
    float vy = 0.0;   // global Vy (m/s)
    float heading;
    float distance;
};
