# Motor Control System

Real-time DC motor control system with IMU-based anomaly detection and 
dual-direction hysteresis compensation, built on Yocto Linux with custom 
kernel drivers for TI J722S SoC.

![Demo](docs/demo.gif)

## Overview

This project was developed as part of an industrial internship and implements 
a complete motor control pipeline — from low-level kernel drivers to a 
real-time monitoring application with terminal UI.

The system addresses a common challenge in DC motor control: **hysteresis** — 
the fact that a motor starts rotating at a different PWM threshold than it 
stops. This is solved through dual-direction calibration that builds separate 
baseline profiles for acceleration and deceleration.

## Features

- Dual-direction calibration with automatic `start_pwm` detection
- Real-time anomaly detection using z-score analysis on speed, vibration and temperature
- Low-pass filtered IMU data to reduce noise
- Multithreaded UI with ASCII animation and color-coded status
- Grace period system to suppress false alarms during speed transitions
- Emergency stop on critical sensor failure
- CSV logging for post-analysis

## Hardware

| Component | Description |
|-----------|-------------|
| TI J722S SBC | Main compute board running Yocto Linux |
| DC Motor + Encoder | Actuator with speed feedback |
| MPU-6050 | 6-axis IMU for vibration and temperature monitoring |
| L298N | Motor driver board |
| 2x Li-ion 18650 | Power supply |

## Project Structure
```
├── src/
│   ├── main.c        # Main control loop, UI, sensor monitoring
│   └── calib.c       # Dual-direction motor calibration
├── drivers/
│   ├── motor_driver.c   # Kernel PWM motor driver
│   └── speed_driver.c   # Encoder speed driver
├── dts/
│   ├── motor.dts     # Device tree overlay for motor
│   └── speed.dts     # Device tree overlay for encoder
├── daemon/
│   └── read_mcu.c    # Userspace IMU daemon (I2C/MPU-6050)
└── assets/
    ├── art_1.txt     # Terminal animation frame 1
    └── art_2.txt     # Terminal animation frame 2
```

## Build
```bash
# Main application
gcc -o main src/main.c -lpthread -lm

# Calibration tool
gcc -o calib src/calib.c -lm

# IMU daemon
gcc -o imu_daemon daemon/read_mcu.c -lm

# Device tree overlays
dtc -@ -I dts -O dtb -o motor.dtbo dts/motor.dts
dtc -@ -I dts -O dtb -o speed.dtbo dts/speed.dts
```

## Usage
```bash
# 1. Load device tree overlays
sudo cp motor.dtbo speed.dtbo /boot/overlays/

# 2. Start IMU daemon
./imu_daemon &

# 3. Run calibration (first time, ~5 min)
./calib

# 4. Start motor control
./main
```

**Controls:** `↑` / `↓` — increase/decrease speed by 5 | `Space` / `S` — emergency stop

## How It Works

### Calibration
The calibration tool runs the motor from 0→100 PWM and back 100→0, 
collecting 30 samples per step for speed, vibration and temperature. 
It automatically detects the minimum PWM at which the motor starts rotating 
(`start_pwm`) and saves separate baseline profiles for each direction.

### Monitoring
During operation, each sensor reading is normalized using z-score against 
the calibration baseline. Readings beyond ±2σ trigger a warning, beyond ±3σ 
trigger an error. After 10–20 consecutive errors an emergency stop is issued.

### Hysteresis Compensation
When decelerating below `start_pwm`, the system immediately cuts power to 
zero instead of trying to maintain low-speed operation where motor behavior 
is unpredictable.

## License
GPL-3.0 — see [LICENSE](LICENSE) for details.