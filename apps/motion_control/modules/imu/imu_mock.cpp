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
}

void Imu::run(void) noexcept {
}

void Imu::init(void) noexcept {
    const auto ec = initialize();
    if (ec) {
        std::cerr << "Imu mock init failed: " << ec.message() << " (code " << ec.value() << ")\n";
    }
}

void Imu::read_all_axis(void) {
    filtered_data = ImuData{0};
}
