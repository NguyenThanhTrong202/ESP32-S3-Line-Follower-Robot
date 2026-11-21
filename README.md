
# ESP32-S3 Smart Line-Following Robot

An embedded robotics project developed using ESP32-S3, featuring an 8-channel analog line sensor array, PWM motor control, track navigation logic, and ultrasonic obstacle detection.

## Overview

This project focuses on developing an autonomous line-following robot capable of navigating different track patterns using an 8-channel analog sensor array.

The ESP32-S3 processes sensor readings, detects the line position, and controls two DC motors through the TB6612FNG motor driver using PID control.

The project is developed to explore embedded firmware, sensor acquisition, motor control, hardware integration, and real-world debugging.

<img width="2752" height="1536" alt="Untitled" src="https://github.com/user-attachments/assets/41512163-9117-4e46-827d-5ec8a72c43f8" />

## Hardware Components

| Component | Description |
|---|---|
| ESP32-S3-WROOM-1 N16R8 | Main microcontroller |
| QTR-8A | 8-channel analog line sensor |
| TB6612FNG | Dual DC motor driver |
| DC Motors | Differential drive system |
| HC-SR04 | Ultrasonic obstacle detection |

## System Features

- Analog sensor data acquisition using ADC
- Line detection and sensor calibration
- Line position analysis
- PWM-based motor speed control
- Straight-line tracking
- Curve and sharp-turn handling
- Ultrasonic obstacle detection
- Line-loss detection and recovery logic

## Hardware Pin Configuration

### TB6612FNG Motor Driver

| ESP32 GPIO | TB6612FNG |
|---|---|
| GPIO 13 | PWMA |
| GPIO 12 | AI2 |
| GPIO 11 | AI1 |
| GPIO 10 | STBY |
| GPIO 9 | BIN1 |
| GPIO 46 | BIN2 |
| GPIO 3 | PWMB |

## Sensor Calibration

Initial sensor readings observed during testing:

- Dark surface: approximately 4095 ADC
- Light surface: approximately 3700 ADC

These values are preliminary and depend on sensor conditions, lighting, and calibration.

## Development Environment

- Arduino IDE
- ESP32 Arduino Core
- C/C++
- Serial Monitor

## Development Roadmap

- [x] ESP32-S3 hardware setup
- [x] Motor driver integration
- [x] Initial sensor testing
- [x] Motor testing
- [x] Sensor calibration
- [x] Improve line detection
- [x] Optimize curve and sharp-turn handling
- [x] Integrate obstacle detection
- [x] Test obstacle response
- [x] Improve line-loss recovery

## Skills & Technologies

C/C++, ESP32-S3, GPIO, ADC, PWM, Embedded Systems, Sensor Interfacing, Motor Control, Firmware Debugging

## Project Status

Currently under development. Firmware and navigation behavior are being continuously tested and improved on the physical robot.

## Author

Nguyễn Thành Trọng
