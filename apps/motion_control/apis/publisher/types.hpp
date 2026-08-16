
struct Encoders{
    uint64_t right_encoder;
    uint64_t left_encoder;
    float right_distance;
    float left_distance;
};

struct Imu {
    // accelerometer in m/s^2
    float ax, ay, az;
    // gyroscope in deg/s
    float gx, gy, gz;
    // magnetometer in microtesla (uT)
    float mx, my, mz; 
    // angle data
    float heading, pitch, roll;
};