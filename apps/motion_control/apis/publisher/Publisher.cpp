#include "Publisher.hpp"

Publisher::Publisher(Encoders& encoders, Imu& imu, Motors& motors) noexcept: 
    _encoders(encoders),
    _imu(imu),
    _motors(motors) {
}

Publisher::~Publisher()= default;

void Publisher::worker(std::stop_token stoken) {
    while(!stoken.stop_requested()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        std::cout << "Publisher: Left Pulses: " << _encoders.get_left_pulses() << ", Right Pulses: " << _encoders.get_right_pulses() << std::endl;
        //mqtt.publish("Left Pulses: " + std::to_string(_encoders.get_left_pulses()) + ", Right Pulses: " + std::to_string(_encoders.get_right_pulses()));
        mqtt.publish("test/topic", "Left Pulses: " + std::to_string(_encoders.get_left_pulses()) + ", Right Pulses: " + std::to_string(_encoders.get_right_pulses()), 1);
    }
}

void Publisher::init(void) noexcept {
    std::cout << "Publisher is initializing..." << std::endl;
        // Connect to broker
        // if (!mqtt.init()) {
        //     std::cerr << "Failed to initialize MQTT connection" << std::endl;
        //     return ;
        // }
        // mqtt.run();

           // Register handlers
    mqtt.registerHandler("devices/001/commands", [](const std::string& payload) {
        //spdlog::info("Command received: {}", payload);
        // Process command...
        std::cout << "Command received: " << payload << std::endl;
    });

    if (mqtt.connect()) {
        mqtt.subscribe("devices/001/commands", 1);
    }

    publisher_thread = std::jthread([this](std::stop_token stoken) {
        worker(stoken);
    });
}

void Publisher::run(void) noexcept {
    std::cout << "Publisher is running..." << std::endl;
}

void Publisher::deInit(void) noexcept {
    std::cout << "Publisher is de-initializing..." << std::endl;
}