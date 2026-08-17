#pragma once

#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <optional>
#include <mutex>
#include <thread>
#include <atomic>

// Minimal ModuleType interface (user provided)
class ModuleType {
public:
    virtual ~ModuleType() = default;
    virtual void init(void) = 0;
    virtual void run(void) = 0;
    virtual void deInit(void) = 0;
};

// GPIO abstraction interface
struct IGpio {
    virtual ~IGpio() = default;
    virtual void setDirection(bool output) = 0; // true = output
    virtual void write(bool level) = 0;
    virtual bool read() = 0;
    // preferEdge: -1 any, 0 falling, 1 rising. Blocks up to timeout; returns true if edge occurred.
    virtual bool waitForEdge(int preferEdge, std::chrono::microseconds timeout) = 0;
};

// Hcsr04 module: extends ModuleType
class Hcsr04Module : public ModuleType {
public:
    struct Config {
        IGpio& triggerPin;
        IGpio& echoPin;
        std::chrono::microseconds triggerPulse{std::chrono::microseconds(10)};
        std::chrono::microseconds measurementTimeout{std::chrono::milliseconds(40)};
        std::chrono::microseconds interMeasurementDelay{std::chrono::milliseconds(60)};
        std::chrono::milliseconds sampleInterval{std::chrono::milliseconds(100)};
        double speedOfSoundMetersPerSecond{343.0};
    };

    explicit Hcsr04Module(const Config& cfg) noexcept;
    ~Hcsr04Module();

    // ModuleType interface
    void init(void) override;   // configure, start internal threads
    void run(void) override;    // optional step function: triggers a single measurement (non-blocking if threads run)
    void deInit(void) override; // stop threads, deinitialize

    // Synchronous single measurement (thread-safe). Blocks up to measurementTimeout.
    std::optional<double> singleShot();

    // Request an async measurement performed by internal worker (non-blocking)
    void triggerAsync();

    // Retrieve last measured distance
    std::optional<double> lastDistance() const;

    // Check if module is running (threads active)
    bool isRunning() const noexcept { return running_.load(); }

private:
    const Config cfg_;

    // Worker thread for periodic sampling
    std::thread workerThread_;
    std::atomic<bool> running_{false};        // true when workerThread_ is active
    std::atomic<bool> stopRequested_{false};  // true when deInit requested

    // Synchronization for tasks and results
    mutable std::mutex ioMutex_;      // protects singleShot hardware access
    std::mutex taskMutex_;
    std::condition_variable taskCv_;
    bool taskRequested_{false};       // signalled to worker to perform measurement immediately

    mutable std::mutex resultMutex_;
    std::optional<double> lastResult_;
    std::chrono::steady_clock::time_point lastMeasurementTime_{};

    // helpers
    static void busyWait(std::chrono::microseconds us) noexcept;
    std::optional<std::chrono::steady_clock::time_point> waitForEdgeAndTimestamp(bool expectRising,
                                                                                 std::chrono::microseconds timeout);

    // internal worker loop
    void workerLoop();
};


#if 0

#include "hcsr04_module.hpp"
#include <iostream>
#include <thread>
#include <chrono>

// Example IGpio mock for demonstration (replace with real implementation)
class MockGpio : public IGpio {
public:
    MockGpio(bool initial = false) : level_(initial) {}
    void setDirection(bool /*output*/) override {}
    void write(bool level) override { level_ = level; }
    bool read() override { return level_; }

    // Simple simulated edge: when waitForEdge called, sleep a bit and toggle level to simulate echo.
    bool waitForEdge(int /*preferEdge*/, std::chrono::microseconds timeout) override {
        // simulate sensor response: for rising edge wait small time, for falling edge wait longer
        using namespace std::chrono_literals;
        std::this_thread::sleep_for(500us); // simulate 500µs before edge
        // toggle to indicate edge occurred
        level_ = !level_;
        (void)timeout;
        return true;
    }

private:
    bool level_;
};

int main() {
    // Create GPIO instances (replace MockGpio with real IGpio implementation)
    MockGpio triggerGpio(false);
    MockGpio echoGpio(false);

    // Configure module
    Hcsr04Module::Config cfg{
        .triggerPin = triggerGpio,
        .echoPin = echoGpio,
        .triggerPulse = std::chrono::microseconds(10),
        .measurementTimeout = std::chrono::milliseconds(40),
        .interMeasurementDelay = std::chrono::milliseconds(60),
        .sampleInterval = std::chrono::milliseconds(200),
        .speedOfSoundMetersPerSecond = 343.0
    };

    // Instantiate module
    Hcsr04Module sensor(cfg);

    // Initialize (configures GPIOs and starts internal worker thread)
    sensor.init();

    // Let worker run for a few samples
    std::this_thread::sleep_for(std::chrono::seconds(1));

    // Read last measured distance
    if (auto d = sensor.lastDistance()) {
        std::cout << "Last distance: " << *d << " m\n";
    } else {
        std::cout << "No measurement available\n";
    }

    // Request an immediate measurement (worker will perform it)
    sensor.triggerAsync();
    std::this_thread::sleep_for(std::chrono::milliseconds(250));
    if (auto d = sensor.lastDistance()) {
        std::cout << "After triggerAsync distance: " << *d << " m\n";
    }

    // Alternatively, perform a blocking single measurement (calls hardware directly)
    if (auto d2 = sensor.singleShot()) {
        std::cout << "singleShot distance: " << *d2 << " m\n";
    } else {
        std::cout << "singleShot timeout or error\n";
    }

    // If you prefer manual stepping without worker thread, deinit then use run()
    sensor.deInit(); // stops worker and cleans up

    // Example of step usage: re-init but don't start worker (not provided in current API).
    // For this design, run() will perform a blocking singleShot if worker not running.
    // Re-initialize then deInit immediately to demonstrate:
    sensor.init();         // starts worker by default
    sensor.deInit();       // stop worker
    // Now call run() which will perform a blocking singleShot because worker not running
    sensor.run();
    if (auto d3 = sensor.lastDistance()) {
        std::cout << "run() distance: " << *d3 << " m\n";
    }

    // Final cleanup
    sensor.deInit();

    return 0;
}



LibGpiodGpio trigger("/dev/gpiochip0", 23, "hcsr04-trigger"); // example offset 23
LibGpiodGpio echo("/dev/gpiochip0", 24, "hcsr04-echo");      // example offset 24

Hcsr04Module::Config cfg{ trigger, echo, /* other params */ };
Hcsr04Module sensor(cfg);
sensor.init();
// ...
sensor.deInit();

#endif