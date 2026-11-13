#pragma once

namespace robot {
    struct PWMCommand {
        double duty{-1.0};   // normalized [-1, 1]
    };
}