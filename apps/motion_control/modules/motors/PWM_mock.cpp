#include "PWM.hpp"
    // channel matches the pwmX index in /sys/class/pwm/pwmchip0/
PWM::PWM(int channel, unsigned int period_ns) noexcept : max_period(period_ns)  {
    duty_path = "/sys/class/pwm/pwmchip0/pwm" + std::to_string(channel) + "/duty_cycle";
}

 PWM::~PWM() {

 }

void PWM::setPWM(float intensity) {
    if (intensity < 0.0f) intensity = 0.0f;
    if (intensity > 1.0f) intensity = 1.0f;
    std::cout << "intensity: " << std::to_string(intensity) << std::endl;

        int duty_ns = static_cast<int>(intensity * max_period);
        
        std::cout << "[PWM] writing "<< std::to_string(duty_ns) << " to file" << std::endl;

}

void PWM::turnOn(void) {
    setPWM(1.0f); 
}

void PWM::turnOff(void) {
    setPWM(0.0f); 
}
