#include "imu.hpp"
#include <iostream>

Imu::Imu() : accel_filter(0.1), gyro_filter(0.1), mag_filter(0.1) {
}

Imu::~Imu() = default;

std::error_code Imu::initialize() {
    return {};
}

void Imu::close() {
}

void Imu::deInit(void) noexcept {
    std::cout << "[Imu] deInit" << std::endl;
}

void Imu::run(void) noexcept {
    std::cout << "[Imu] run" << std::endl;
}

void Imu::init(void) noexcept {
    std::cout << "[Imu] init" << std::endl;
    const auto ec = initialize();
    if (ec) {
        std::cerr << "Imu mock init failed: " << ec.message() << " (code " << ec.value() << ")\n";
    }
}

std::optional<AllAxes> Imu::read_all() {
    filtered_data = ImuData{0};
}
