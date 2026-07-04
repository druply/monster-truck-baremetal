#include "mpu9250.hpp"
#include "axis_struct.hpp"

using namespace mpu9250;

std::optional<AllAxes> MPU9250::read_all() {
 AllAxes out;

 out.ax = 0.5;
 out.ay = 0.5;
 out.az = 9.8;
 
 return out;
}

std::error_code MPU9250::initialize() {

    return std::error_code(); // success
}

MPU9250::~MPU9250() { close(); }

MPU9250::MPU9250(const Config& cfg) : cfg_(cfg) {}



void MPU9250::close() {

}
