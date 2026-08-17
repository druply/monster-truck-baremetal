#include "Publisher.hpp"
#include <iostream>

Publisher::Publisher(Encoders& encoders, Imu& imu, Motors& motors) noexcept
    : _encoders(encoders), _imu(imu), _motors(motors) {
}

Publisher::~Publisher() = default;

void Publisher::worker(std::stop_token stoken) {
    while (!stoken.stop_requested()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

void Publisher::init(void) noexcept {
    std::cout << "[Publisher] init" << std::endl;
}

void Publisher::run(void) noexcept {
    std::cout << "[Publisher] run" << std::endl;
}

void Publisher::deInit(void) noexcept {
    std::cout << "[Publisher] deInit" << std::endl;
}
