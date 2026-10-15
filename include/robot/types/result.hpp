// result.hpp
#pragma once
#include <expected>
#include <string>

namespace robot
{
    enum class Module
    {
        Imu,
        Encoder,
        Motor,
        Lidar
    };

    enum class ErrorCode
    {
        NotInitialized,
        IoFailure,
        Timeout,
        InvalidArgument,
        DeviceNotFound,
        ChecksumMismatch,
    };

    struct Error
    {
        Module module;
        ErrorCode code;
        std::string message;
    };
    template <typename T>
    using Result = std::expected<T, Error>;
}