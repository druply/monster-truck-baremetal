// result.hpp
#pragma once
#include <expected>
#include <string>

namespace robot {
    enum class Error {
        NotInitialized,
        IoFailure,
        Timeout,
        InvalidArgument,
        DeviceNotFound,
        ChecksumMismatch,
    };
    template <typename T>
    using Result = std::expected<T, Error>;
}