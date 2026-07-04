#include "libgpiod_gpio.hpp"
#include <cstring>
#include <chrono>

LibGpiodGpio::LibGpiodGpio(const std::string &chipPath, unsigned int lineOffset, const std::string &consumer)
    : chipPath_(chipPath), lineOffset_(lineOffset), consumer_(consumer)
{
    openChipAndLine();
}

LibGpiodGpio::~LibGpiodGpio()
{
    try {
        releaseLine();
        if (chip_) {
            gpiod_chip_close(chip_);
            chip_ = nullptr;
        }
    } catch (...) {
        // Suppress exceptions in destructor
    }
}

void LibGpiodGpio::openChipAndLine()
{
    chip_ = gpiod_chip_open(chipPath_.c_str());
    if (!chip_) throw GpioError("Failed to open gpio chip: " + chipPath_ + " (" + std::strerror(errno) + ")");

    line_ = gpiod_chip_get_line(chip_, lineOffset_);
    if (!line_) {
        gpiod_chip_close(chip_);
        chip_ = nullptr;
        throw GpioError("Failed to get line " + std::to_string(lineOffset_) + " from " + chipPath_);
    }

    // Do not request direction yet; wait for setDirection call.
    isOutput_ = false;
}

void LibGpiodGpio::releaseLine()
{
    if (line_) {
        gpiod_line_release(line_);
        line_ = nullptr;
    }
}

void LibGpiodGpio::setDirection(bool output)
{
    if (!chip_ || !line_) throw GpioError("GPIO not initialized");

    // If already in desired direction, do nothing (but ensure line reserved appropriately)
    if (isOutput_ == output) return;

    // Release previous request (if any)
    if (line_) gpiod_line_release(line_);

    // Re-acquire line with appropriate direction
    if (output) {
        int ret = gpiod_line_request_output(line_, consumer_.c_str(), 0);
        if (ret < 0) throw GpioError("Failed to request line as output");
        isOutput_ = true;
    } else {
        int ret = gpiod_line_request_both_edges_events(line_, consumer_.c_str());
        if (ret < 0) {
            // As fallback, request input (no events) to allow read() polling
            ret = gpiod_line_request_input(line_, consumer_.c_str());
            if (ret < 0) throw GpioError("Failed to request line as input");
            isOutput_ = false;
        } else {
            isOutput_ = false;
        }
    }
}

void LibGpiodGpio::write(bool level)
{
    if (!line_) throw GpioError("GPIO line not initialized");
    if (!isOutput_) {
        // Try to reconfigure as output
        setDirection(true);
    }
    int ret = gpiod_line_set_value(line_, level ? 1 : 0);
    if (ret < 0) throw GpioError("Failed to set line value");
}

bool LibGpiodGpio::read()
{
    if (!line_) throw GpioError("GPIO line not initialized");
    // If currently requested as output, read still possible via value query
    int val = gpiod_line_get_value(line_);
    if (val < 0) throw GpioError("Failed to read line value");
    return val != 0;
}

bool LibGpiodGpio::waitForEdge(int preferEdge, std::chrono::microseconds timeout)
{
    if (!line_) throw GpioError("GPIO line not initialized");

    // Ensure line is requested for EDGE events. If currently output, we cannot wait for edge.
    if (isOutput_) {
        // no edge possible if output; return false immediately
        return false;
    }

    // If line was requested only as input (no events), we can fallback to polling.
    // Determine if events are supported by checking line info capabilities.
    // gpiod_line_event_wait handles both blocking and timeout.
    struct gpiod_line_event event;
    struct timespec ts;
    ts.tv_sec = std::chrono::duration_cast<std::chrono::seconds>(timeout).count();
    ts.tv_nsec = std::chrono::duration_cast<std::chrono::nanoseconds>(timeout).count() % 1000000000L;

    int ret = gpiod_line_event_wait(line_, &ts);
    if (ret < 0) {
        // error
        throw GpioError("gpiod_line_event_wait failed");
    } else if (ret == 0) {
        // timeout
        return false;
    }

    // event available; read it
    ret = gpiod_line_event_read(line_, &event);
    if (ret < 0) throw GpioError("gpiod_line_event_read failed");

    // Map gpiod event to preferEdge
    if (preferEdge == -1) return true;
    if (preferEdge == 1 && event.event_type == GPIOD_LINE_EVENT_RISING) return true;
    if (preferEdge == 0 && event.event_type == GPIOD_LINE_EVENT_FALLING) return true;

    // If different edge arrived, continue waiting up to remaining timeout
    // For simplicity, treat as occurrence only when matching; return false to indicate no matching edge in timeout.
    return false;
}
