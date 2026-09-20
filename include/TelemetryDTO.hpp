#include <string>

struct TelemetryDTO {
    std::string id;
    std::string timestamp;
    int device_id;
    std::string metric;
    int value;
    std::string unit;
    std::string quality;
};