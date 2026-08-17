#pragma once

#include <chrono>
#include <string>
#include <stdexcept>
#include "hcsr04_module.hpp" // for IGpio definition
#include <gpiod.h>

class GpioError : public std::runtime_error {
public:
    explicit GpioError(const std::string &msg) : std::runtime_error(msg) {}
};

// Concrete IGpio implementation using libgpiod
class LibGpiodGpio : public IGpio {
public:
    // chipPath: e.g., "/dev/gpiochip0"
    // lineOffset: numeric offset on the chip
    // consumer: optional consumer label shown in kernel
    LibGpiodGpio(const std::string &chipPath, unsigned int lineOffset, const std::string &consumer = "hcsr04");
    ~LibGpiodGpio() override;

    void setDirection(bool output) override;
    void write(bool level) override;
    bool read() override;
    bool waitForEdge(int preferEdge, std::chrono::microseconds timeout) override;

private:
    std::string chipPath_;
    unsigned int lineOffset_;
    std::string consumer_;
    gpiod_chip *chip_{nullptr};
    gpiod_line *line_{nullptr};
    bool isOutput_{false};

    void openChipAndLine();
    void releaseLine();
};
