#pragma once

namespace robot {
    struct PWMCommand {
        double duty{};   // normalized [0, 1]; direction is controlled separately
    };
}