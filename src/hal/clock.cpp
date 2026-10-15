#include "utils/clock.hpp"
#include <chrono>

namespace utils
{

    [[nodiscard]] robot::Timestamp Clock::monotonic_time_ms() const noexcept
    {
        using namespace std::chrono;
        auto now = steady_clock::now();
        return duration_cast<milliseconds>(now.time_since_epoch()).count();
    }

}