#include "hcsr04_module.hpp"
#include <thread>
#include <stdexcept>

// Constructor
Hcsr04Module::Hcsr04Module(const Config& cfg) noexcept
    : cfg_(cfg)
{
    // minimal setup; full start in init()
}

// Destructor
Hcsr04Module::~Hcsr04Module()
{
    deInit();
}

// init: configure GPIOs, reset state, start worker thread
void Hcsr04Module::init(void)
{
    stopRequested_.store(false);

    // Configure pins
    cfg_.triggerPin.setDirection(true);
    cfg_.echoPin.setDirection(false);
    cfg_.triggerPin.write(false);

    {
        std::lock_guard lk(resultMutex_);
        lastResult_.reset();
        lastMeasurementTime_ = std::chrono::steady_clock::time_point{};
    }

    // Start worker thread if not already running
    bool expected = false;
    if (running_.compare_exchange_strong(expected, true)) {
        workerThread_ = std::thread(&Hcsr04Module::workerLoop, this);
    }
}

// run: optional step function. If worker thread is active, this requests an immediate measurement
// If worker is not active, run() will perform a single synchronous measurement.
void Hcsr04Module::run(void)
{
    if (isRunning()) {
        triggerAsync();
    } else {
        // perform blocking single measurement
        singleShot();
    }
}

// deInit: request stop, join worker thread, deconfigure if needed
void Hcsr04Module::deInit(void)
{
    stopRequested_.store(true);

    // wake worker
    {
        std::lock_guard lk(taskMutex_);
        taskRequested_ = false;
    }
    taskCv_.notify_all();

    if (workerThread_.joinable()) {
        workerThread_.join();
    }
    running_.store(false);

    // Optionally set trigger low and release resources
    try {
        cfg_.triggerPin.write(false);
    } catch (...) {
        // swallow errors in destructor context
    }
}

// Trigger an async measurement inside worker loop
void Hcsr04Module::triggerAsync()
{
    {
        std::lock_guard lk(taskMutex_);
        taskRequested_ = true;
    }
    taskCv_.notify_all();
}

// Return last measured value
std::optional<double> Hcsr04Module::lastDistance() const
{
    std::lock_guard lk(resultMutex_);
    return lastResult_;
}

// Synchronous single measurement implementation
std::optional<double> Hcsr04Module::singleShot()
{
    std::unique_lock lk(ioMutex_);

    // Enforce inter-measurement delay
    auto now = std::chrono::steady_clock::now();
    if (lastMeasurementTime_.time_since_epoch().count() != 0) {
        auto since = std::chrono::duration_cast<std::chrono::microseconds>(now - lastMeasurementTime_);
        if (since < cfg_.interMeasurementDelay) {
            std::this_thread::sleep_for(cfg_.interMeasurementDelay - since);
        }
    }

    // Trigger pulse
    cfg_.triggerPin.write(true);
    busyWait(cfg_.triggerPulse);
    cfg_.triggerPin.write(false);

    // Wait for rising edge
    auto timeout = cfg_.measurementTimeout;
    auto start_ts = waitForEdgeAndTimestamp(true, timeout);
    if (!start_ts) return std::nullopt;

    // Remaining timeout for falling edge
    auto afterRising = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::steady_clock::now() - *start_ts);
    auto remaining = timeout > afterRising ? (timeout - afterRising) : std::chrono::microseconds(0);
    auto end_ts = waitForEdgeAndTimestamp(false, remaining);
    if (!end_ts) return std::nullopt;

    lastMeasurementTime_ = std::chrono::steady_clock::now();

    auto pulseWidth = std::chrono::duration_cast<std::chrono::microseconds>(*end_ts - *start_ts);
    double seconds = pulseWidth.count() / 1'000'000.0;
    double distance = (seconds * cfg_.speedOfSoundMetersPerSecond) / 2.0;

    {
        std::lock_guard lk(resultMutex_);
        lastResult_ = distance;
    }

    return distance;
}

// Worker loop for periodic sampling and async triggers
void Hcsr04Module::workerLoop()
{
    auto interval = cfg_.sampleInterval;
    while (!stopRequested_.load()) {
        // wait for either interval or task request
        {
            std::unique_lock lk(taskMutex_);
            taskCv_.wait_for(lk, interval, [this] { return taskRequested_ || stopRequested_.load(); });
            if (stopRequested_.load()) break;
            taskRequested_ = false;
        }

        // perform measurement (blocking)
        auto res = singleShot();

        // store result (already stored by singleShot), but ensure visibility
        if (res) {
            std::lock_guard lk(resultMutex_);
            lastResult_ = res;
        }
    }
}

// High-resolution busy-wait for short durations (10us)
void Hcsr04Module::busyWait(std::chrono::microseconds us) noexcept
{
    auto start = std::chrono::steady_clock::now();
    while (std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - start) < us) {
        asm volatile("" ::: "memory");
    }
}

// Use the IGpio backend's blocking waitForEdge when available; return timestamp on edge.
std::optional<std::chrono::steady_clock::time_point> Hcsr04Module::waitForEdgeAndTimestamp(bool expectRising,
                                                                                           std::chrono::microseconds timeout)
{
    int prefer = expectRising ? 1 : 0;
    if (cfg_.echoPin.waitForEdge(prefer, timeout)) {
        return std::chrono::steady_clock::now();
    }
    return std::nullopt;
}
