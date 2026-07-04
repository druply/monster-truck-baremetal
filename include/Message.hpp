#include <memory>
#include <cstdint>

// Message payload is entirely immutable (const)
struct TelemetryMessage {
    const uint64_t left_pulses;
    const uint64_t right_pulses;
    const bool system_fault;
};

using TelemetryMsgPtr = std::shared_ptr<const TelemetryMessage>;
