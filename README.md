
# ESP32-S3 Smart Line-Following Robot

An embedded robotics project developed using ESP32-S3, featuring an 8-channel analog line sensor array, PWM motor control, track navigation logic, and ultrasonic obstacle detection.

## Overview

This project focuses on developing an autonomous line-following robot capable of navigating different track patterns using an 8-channel analog sensor array.

The ESP32-S3 processes sensor readings, detects the line position, and controls two DC motors through the TB6612FNG motor driver using PID control.

The project is developed to explore embedded firmware, sensor acquisition, motor control, hardware integration, and real-world debugging.

<img width="2752" height="1536" alt="Untitled" src="https://github.com/user-attachments/assets/41512163-9117-4e46-827d-5ec8a72c43f8" />

## Hardware Configuration

### Hardware Components

| Component | Description |
|---|---|
| ESP32-S3-WROOM-1 N16R8 | Main microcontroller |
| QTR-8A | 8-channel analog reflectance sensor array for line detection |
| TB6612FNG | Dual DC motor driver |
| HC-SR04 | Ultrasonic distance sensor for obstacle detection |
| DC Motors | Differential drive system |

### Pin Mapping

#### 1. QTR-8A Line Sensor

The QTR-8A sensor array provides 8 analog channels for detecting line position and tracking the path.

| QTR-8A Pin | ESP32-S3 GPIO |
|---|---:|
| D1 | GPIO 18 |
| D2 | GPIO 17 |
| D3 | GPIO 16 |
| D4 | GPIO 15 |
| D5 | GPIO 7 |
| D6 | GPIO 6 |
| D7 | GPIO 5 |
| D8 | GPIO 4 |
| VCC | 3.3V |
| GND | GND |

#### 2. HC-SR04 Ultrasonic Sensor

The HC-SR04 is used to measure the distance between the robot and obstacles.

| HC-SR04 Pin | ESP32-S3 GPIO |
|---|---:|
| TRIG | GPIO 41 |
| ECHO | GPIO 40 |
| VCC | 5V |
| GND | GND |

> **Note:** The HC-SR04 ECHO pin outputs a 5V signal. A voltage divider or logic-level converter should be used to protect the ESP32-S3 GPIO from 5V.

#### 3. TB6612FNG Motor Driver

The TB6612FNG controls the direction and speed of the two DC motors using GPIO signals and PWM.

| TB6612FNG Pin | ESP32-S3 GPIO | Function |
|---|---:|---|
| PWMA | GPIO 13 | Motor A speed control (PWM) |
| AI2 | GPIO 12 | Motor A direction |
| AI1 | GPIO 11 | Motor A direction |
| STBY | GPIO 10 | Driver standby control |
| BIN1 | GPIO 9 | Motor B direction |
| BIN2 | GPIO 46 | Motor B direction |
| PWMB | GPIO 3 | Motor B speed control (PWM) |

### System Hardware Overview

- **Line Detection:** QTR-8A analog sensor array
- **Motor Control:** TB6612FNG dual motor driver
- **Obstacle Detection:** HC-SR04 ultrasonic sensor
- **Main Controller:** ESP32-S3-WROOM-1 N16R8
- **Drive System:** Two DC motors

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

This project has been completed and archived. No further development is planned.

Project Achievement: Top 15 / 54 teams
## Author

Nguyễn Thành Trọng
