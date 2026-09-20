#pragma once
#include <string>
#include <vector>
#include "hardware/imu/imu.hpp"
#include "hardware/encoders/encoders.hpp"
#include <boost/json.hpp>

class parseToJSON
{
public:
    static std::string serializeImuData(const ImuData &imuData)
    {
        std::string timestamp = std::to_string(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count());

        json::array imu_data = {
            json::object{{"timestamp", timestamp},
                         {"metric", "acceleration-x"},
                         {"value", imuData.ax},
                         {"unit", "m/s^2"},
                         {"quality", "good"}},

            json::object{{"timestamp", timestamp},
                         {"metric", "acceleration-y"},
                         {"value", imuData.ay},
                         {"unit", "m/s^2"},
                         {"quality", "good"}},

            json::object{{"timestamp", timestamp},
                         {"metric", "acceleration-z"},
                         {"value", imuData.az},
                         {"unit", "m/s^2"},
                         {"quality", "good"}}

            json::object{{"timestamp", timestamp},
                         {"metric", "gyrometer-x"},
                         {"value", imuData.gx},
                         {"unit", "deg/s"},
                         {"quality", "good"}},
            json::object{{"timestamp", timestamp},
                         {"metric", "gyrometer-y"},
                         {"value", imuData.gy},
                         {"unit", "deg/s"},
                         {"quality", "good"}},
            json::object{{"timestamp", timestamp},
                         {"metric", "gyrometer-z"},
                         {"value", imuData.gz},
                         {"unit", "deg/s"},
                         {"quality", "good"}}

            json::object{{"timestamp", timestamp},
                         {"metric", "magnetometer-x"},
                         {"value", imuData.mx},
                         {"unit", "uT"},
                         {"quality", "good"}},
            json::object{{"timestamp", timestamp},
                         {"metric", "magnetometer-y"},
                         {"value", imuData.my},
                         {"unit", "uT"},
                         {"quality", "good"}},
            json::object{{"timestamp", timestamp},
                         {"metric", "magnetometer-z"},
                         {"value", imuData.mz},
                         {"unit", "uT"},
                         {"quality", "good"}}};

        return json::serialize(imu_data);
    }

    static std::string serializeEncoderData(const uint64_t &left_pulses, const uint64_t &right_pulses)
    {
        json::array encoder_data = {
            json::object{{"timestamp", std::to_string(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count())},
                         {"metric", "left_encoder_pulses"},
                         {"value", left_pulses},
                         {"unit", "pulses"},
                         {"quality", "good"}},

            json::object{{"timestamp", std::to_string(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count())},
                         {"metric", "right_encoder_pulses"},
                         {"value", right_pulses},
                         {"unit", "pulses"},
                         {"quality", "good"}}};

        return json::serialize(encoder_data);
    }
};