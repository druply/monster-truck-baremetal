#include <cstdint>
#include <memory>

// Define system-wide topics as cheap numeric tags
enum class Topic : uint16_t {
    EncoderData,   // Maps conceptually to "encoder/data"
    MotorAction    // Maps conceptually to "motor/action"
};

// --- Payload Definitions ---
struct EncoderDataPayload {
    uint64_t left_pulses{0};
    uint64_t right_pulses{0};
};

struct MotorActionPayload {
    float left_motor_effort{0.0f};  // Range: -1.0 to 1.0
    float right_motor_effort{0.0f};
};
