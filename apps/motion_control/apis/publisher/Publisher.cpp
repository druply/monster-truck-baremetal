#include "Publisher.hpp"

explicit Publisher::Publisher(Encoders& encoders, Imu& imu, Motors& motors): _encoders(encoders), _imu(imu), _motors(motors) {

}

Publisher::~Publisher()= default;

void Publisher::worker(std::stop_token stoken) {
    while(!stoken.stop_requested()) {
        
    }
}

void Publisher::init(void) noexcept {
    std::jthread t([this](std::stop_token stoken) {
        worker(stoken);
    });
}

void Publisher::run(void) noexcept {

}

void Publisher::deInit(void) noexcept {
    
}