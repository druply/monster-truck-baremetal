/* 
This example assumes you already configured the pwm using the service file and enabled the board on /boot/firmware/config.txt
Since the systemd service has already handled exporting and setting the frequency, 
your code simply needs to write new values to the duty_cycle file to change brightness or toggle the LED on/off. 

*/
#include <iostream>
#include <fstream>
#include <string>
#include <thread>
#include <chrono>

class PWM {
    std::string duty_path;
    unsigned int max_period;
public:
    // channel matches the pwmX index in /sys/class/pwm/pwmchip0/
    PWM(int channel, unsigned int period_ns) noexcept;
    ~PWM();

    // Set brightness from 0.0 (off) to 1.0 (full brightness)
    void setPWM(float intensity);

    void turnOn(void);
    void turnOff(void);
};

/*
int main() {
    // 20,000,000 ns matches the 50Hz set in your systemd service
    PWMLED led0(0, 20000000); 

    std::cout << "Starting LED pattern..." << std::endl;

    for (int i = 0; i < 3; ++i) {
        led0.turnOn();
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        led0.turnOff();
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    std::cout << "Fading out..." << std::endl;
    for (float b = 1.0f; b >= 0.0f; b -= 0.05f) {
        led0.setBrightness(b);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    return 0;
}
*/