#!/bin/bash
# Path to your PCA9685 PWM chip
# Create the script: sudo nano /usr/bin/pca9685-init.sh
PWM_PATH="/sys/class/pwm/pwmchip0"

if [ ! -d "$PWM_PATH" ]; then
echo "pwmchip not found at $PWM_PATH" >&2
exit 1
fi

# Wait for the driver to be fully loaded
sleep 2

# Export channels 0 to 15 if not already exported
for i in {0..15}; do
    if [ ! -d "${PWM_PATH}/pwm${i}" ]; then
        echo $i > "${PWM_PATH}/export"
    fi
    
    # Set default 50Hz frequency (20,000,000 ns period)
    #echo 20000000 > "${PWM_PATH}/pwm${i}/period"
    # Set default 300Hz frequency (3,333,333.33 ns period)
    echo 3333333 > "${PWM_PATH}/pwm${i}/period"
    # Set initial duty cycle to 0 (off)
    echo 0 > "${PWM_PATH}/pwm${i}/duty_cycle"
    # Enable the channel
    echo 1 > "${PWM_PATH}/pwm${i}/enable"
done

# Optional: Set group permissions so your C++ app can run without sudo
chown -R root:i2c ${PWM_PATH}/*
chmod -R g+rw ${PWM_PATH}/*