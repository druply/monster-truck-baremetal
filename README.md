# Monster Truck Baremetal Robot

The project uses C++23. `robot_core` contains hardware-independent odometry,
`robot_runtime` owns the lifecycle and data flow, and `robot_pca9685` provides
the Linux I2C output driver. The `Robot` executable currently uses simulated
devices so it can be built and exercised on a development machine.

## Build and test

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build
ctest --test-dir build --output-on-failure
./build/bin/Robot
```

## PCA9685 PWM

`robot::drivers::Pca9685Pwm` controls one PCA9685 output. Configure the Linux
I2C device (typically `/dev/i2c-1`), I2C address (typically `0x40`, subject to
board jumpers), PWM frequency, and channel `0` through `15`. `PWMCommand::duty`
is a finite normalized value in `[0, 1]`; invalid values return an error rather
than being silently clamped. Initialization puts that channel into the off
state, and shutdown attempts to turn it off.
After initialization, `Robot::set_motor_output` sends a duty command to either
the configured rear or front output. `Robot::spin_once` shuts down both outputs
if a sensor, odometry, or actuator operation fails.

The PCA9685 supplies PWM signals; it does not itself drive or reverse a motor.
Use a suitable motor driver/H-bridge for motor current and direction control.
Direction must be controlled through its direction input(s), which can be
separate GPIOs or separately configured PCA9685 channels. The current PWM
interface controls only one PWM output and does not implement direction.

The encoder interface is currently polled synchronously by `Robot::spin_once`,
so one thread owns each device. Encoder tick counters are unsigned and
cumulative; reverse-direction counting requires a quadrature encoder or an
external direction signal. Odometry currently reports wheel distances and
velocities, not a fused pose.

## Raspberry Pi notes

Build and run the real PCA9685 driver on Linux with I2C enabled and the process
permitted to access `/dev/i2c-*`. The default executable intentionally remains
a simulator; production hardware assembly should inject `Pca9685Pwm` instances
through `robot::app::Hardware`. The PCA9685 channels share one board PWM
frequency, so configure all output instances on that board consistently.

## Sanitizers and compiler hardening

Compiler hardening is enabled by default. GNU and Clang builds use stack
protection, and Release/RelWithDebInfo builds also enable `_FORTIFY_SOURCE=2`.
MSVC builds use `/GS` and `/sdl`.

Use separate build directories for sanitizers:

```sh
cmake -S . -B build-asan -DCMAKE_BUILD_TYPE=Debug -DENABLE_ASAN=ON
cmake --build build-asan
ctest --test-dir build-asan --output-on-failure

cmake -S . -B build-tsan -DCMAKE_BUILD_TYPE=Debug -DENABLE_TSAN=ON
cmake --build build-tsan
ctest --test-dir build-tsan --output-on-failure
```

ASan and TSan cannot be enabled in the same build. Sanitizers find some memory
errors and data races when affected code runs, but do not replace tests or
hardware validation.
