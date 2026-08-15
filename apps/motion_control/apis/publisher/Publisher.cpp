#include "Publisher.hpp"

explicit Publisher::Publisher(Encoders& encoders, Imu& imu, Motors& motors): _encoders(encoders), _imu(imu), _motors(motors) {

}

Publisher::~Publisher()= default;

void Publisher::init(void) noexcept {

}

void Publisher::run(void) noexcept {

}

void Publisher::deInit(void) noexcept {
    
}