#include "ModuleType.hpp"
#include "modules/encoders/encoders.hpp"
#include "modules/motors/motors.hpp"
#include "modules/imu/imu.hpp"

class Publisher: public ModuleType {
    Encoders& _encoders;
    Imu& _imu;
    Motors& _motors;

    public:
        explicit Publisher(Encoders& encoders, Imu& imu, Motors& motors) noexcept;
        ~Publisher();        
        void init(void) noexcept override;
		void run(void) noexcept override;
		void deInit(void) noexcept override;
};