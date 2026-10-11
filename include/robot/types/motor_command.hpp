// motor_command.hpp
#pragma once

namespace robot {
    struct MotorCommand {
        double duty{-1.0};   // normalized [-1, 1]
    };
}