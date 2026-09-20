#include "Publisher.hpp"

#include "TelemetryDTO.hpp"
#include "parseToJSON.hpp"

namespace json = boost::json;

PubSub::PubSub(Encoders &encoders, Imu &imu, Motors &motors) noexcept : _encoders(encoders),
                                                                        _imu(imu),
                                                                        _motors(motors)
{
}

PubSub::~PubSub() = default;

void PubSub::worker(std::stop_token stoken)
{
    while (!stoken.stop_requested())
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        std::vector<TelemetryDTO> telemetryData;
        TelemetryDTO accelerometerXData;
        TelemetryDTO accelerometerYData;
        TelemetryDTO accelerometerZData;

        //std::cout << "Publisher: Left Pulses: " << _encoders.get_left_pulses() << ", Right Pulses: " << _encoders.get_right_pulses() << std::endl;
        // mqtt.publish("Left Pulses: " + std::to_string(_encoders.get_left_pulses()) + ", Right Pulses: " + std::to_string(_encoders.get_right_pulses()));
        ImuData imuData = _imu.getImuData();
        std::string imuJson = parseToJSON::serializeImuData(imuData);

        std::string encoderJson = parseToJSON::serializeEncoderData(_encoders.get_left_pulses(), _encoders.get_right_pulses());

        mqtt.publish("devices/001/imu", imuJson, 1);
        mqtt.publish("devices/001/encoders", encoderJson, 1);
        // mqtt.publish("test/topic", "Left Pulses: " + std::to_string(_encoders.get_left_pulses()) + ", Right Pulses: " + std::to_string(_encoders.get_right_pulses()), 1);
    }
}

void PubSub::init(void) noexcept
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

void PubSub::run(void) noexcept
{
    std::cout << "Publisher is running..." << std::endl;
}

void PubSub::deInit(void) noexcept
{
    std::cout << "Publisher is de-initializing..." << std::endl;
}