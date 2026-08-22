#include "ModuleType.hpp"
#include <thread>
#include "hardware/encoders/encoders.hpp"
#include "hardware/motors/motors.hpp"
#include "hardware/imu/imu.hpp"
#include "production_mqtt_client.hpp"

class Publisher : public ModuleType
{
    Encoders &_encoders;
    Imu &_imu;
    Motors &_motors;
    std::jthread publisher_thread;
    void worker(std::stop_token stoken);
    // MQTT mqtt{"tcp://localhost:1883", "paho_test_client", "test/topic"};
    ProductionMQTTClient mqtt{"tcp://localhost:1883", "test"};

public:
    explicit Publisher(Encoders &encoders, Imu &imu, Motors &motors) noexcept;
    ~Publisher();
    void init(void) noexcept override;
    void run(void) noexcept override;
    void deInit(void) noexcept override;
};