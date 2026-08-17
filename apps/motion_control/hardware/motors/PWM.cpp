#include "PWM.hpp"
    // channel matches the pwmX index in /sys/class/pwm/pwmchip0/
PWM::PWM(int channel, unsigned int period_ns) noexcept : max_period(period_ns) {
    duty_path = "/sys/class/pwm/pwmchip0/pwm" + std::to_string(channel) + "/duty_cycle";
}

 PWM::~PWM() {

 }

void PWM::setPWM(float intensity) {
    if (intensity < 0.0f) intensity = 0.0f;
    if (intensity > 1.0f) intensity = 1.0f;

     //std::cout << "intensity: " << std::to_string(intensity) << std::endl;
     //std::cout << "max_period: " << std::to_string(max_period) << std::endl;

        int duty_ns = static_cast<int>(intensity * static_cast<float>(max_period));
        //std::cout << "writing period: " << duty_ns << std::endl;
        std::ofstream file(duty_path);
        if (file.is_open()) {
            file << duty_ns;
            file.close();
        } else {
            std::cerr << "Error: Could not open " << duty_path << ". Check permissions!" << std::endl;
        }

}

void PWM::turnOn(void) {
    setPWM(1.0f); 
}

void PWM::turnOff(void) {
    setPWM(0.0f); 
}
