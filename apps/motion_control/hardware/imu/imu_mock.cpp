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
    filtered_data = ImuData{0.2f, 0.4f, 9.81f, 1.3f, 0.73f, 0.1203f, 30.0f, 1.220f, 40.323f, 25.43f, 1};
}

ImuData Imu::getImuData(void)
{
    return filtered_data;
}

void Imu::init(void) noexcept {
    std::cout << "[Imu] init" << std::endl;
    const auto ec = initialize();
    if (ec) {
        std::cerr << "Imu mock init failed: " << ec.message() << " (code " << ec.value() << ")\n";
    }
}

std::optional<AllAxes> Imu::read_all() {
    filtered_data = ImuData{0.0f, 0.0f, 9.81f, 0.0f, 0.0f, 0.0f, 30.0f, 0.0f, 40.0f, 25.0f, 1};
}
