#include "Publisher.hpp"
#include <boost/json.hpp>

namespace json = boost::json;

Publisher::Publisher(Encoders &encoders, Imu &imu, Motors &motors) noexcept : _encoders(encoders),
                                                                              _imu(imu),
                                                                              _motors(motors)
{
}

Publisher::~Publisher() = default;

void Publisher::worker(std::stop_token stoken)
{
    while (!stoken.stop_requested())
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        std::cout << "Publisher: Left Pulses: " << _encoders.get_left_pulses() << ", Right Pulses: " << _encoders.get_right_pulses() << std::endl;
        // mqtt.publish("Left Pulses: " + std::to_string(_encoders.get_left_pulses()) + ", Right Pulses: " + std::to_string(_encoders.get_right_pulses()));
        ImuData imuData = _imu.getImuData();
        json::object imu_data = {
            {"accel_x", imuData.ax},
            {"accel_y", imuData.ay},
            {"accel_z", imuData.az},
            {"gyro_x", imuData.gx},
            {"gyro_y", imuData.gy},
            {"gyro_z", imuData.gz},
            {"mag_x", imuData.mx},
            {"mag_y", imuData.my},
            {"mag_z", imuData.mz}
        };
        json::object encoder_data = {
            {"right", _encoders.get_right_pulses()},
            {"left", _encoders.get_left_pulses()}
        };

        MotorPwm_t motors_pwm = _motors.getMotorsPwm();

        json::object motors_data = {
            {"steering_angle", _motors.getSteeringAngle()},
            {"motors_pwm", {
                {"right", motors_pwm.right},
                {"left", motors_pwm.left}
            }}
        };
        json::object data = {
            {"imu", imu_data},
            {"encoders", encoder_data},
            {"motors", motors_data}
        };

        std::string jsonString = json::serialize(data);
        mqtt.publish("devices/001/sensor_data", jsonString, 1);
        //mqtt.publish("test/topic", "Left Pulses: " + std::to_string(_encoders.get_left_pulses()) + ", Right Pulses: " + std::to_string(_encoders.get_right_pulses()), 1);

    }
}

void Publisher::init(void) noexcept
{
    std::cout << "Publisher is initializing..." << std::endl;
    // Register handlers
    mqtt.registerHandler("devices/001/commands", [](const std::string &payload)
                         {
        //spdlog::info("Command received: {}", payload);
        // Process command...
        std::cout << "Command received: " << payload << std::endl; });

    if (mqtt.connect())
    {
        mqtt.subscribe("devices/001/commands", 1);
    }
    else
    {
        std::cerr << "Failed to connect to MQTT broker" << std::endl;
    }

    publisher_thread = std::jthread([this](std::stop_token stoken)
                                    { worker(stoken); });
}

void Publisher::run(void) noexcept
{
    std::cout << "Publisher is running..." << std::endl;
}

void Publisher::deInit(void) noexcept
{
    std::cout << "Publisher is de-initializing..." << std::endl;
}