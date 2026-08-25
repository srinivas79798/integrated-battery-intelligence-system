# Integrated Battery Intelligence System

An ESP32-based embedded and IoT system for real-time four-cell battery monitoring, analytics, safety protection, fault handling, LCD diagnostics, and Blynk cloud monitoring.

## Project Overview

The Integrated Battery Intelligence System combines battery intelligence, event-driven safety protection, embedded HMI, fault-tolerant runtime management, cloud telemetry, and an executive IoT dashboard into a single system.

The system monitors a simulated four-cell lithium battery pack and detects abnormal operating conditions while automatically controlling protection outputs.

## Key Features

- Four-cell voltage monitoring
- Pack voltage and average voltage calculation
- Cell imbalance calculation
- Weakest and strongest cell identification
- Battery health classification
- Weak-cell detection
- Overvoltage detection
- Sensor fault detection
- Rapid voltage fluctuation detection
- Frozen ADC detection
- Relay mismatch detection
- Automatic relay protection
- Buzzer and LED fault indication
- LCD-based diagnostics
- NORMAL, DEGRADED, FAILSAFE, and SHUTDOWN states
- Non-blocking millis()-based architecture
- Blynk IoT cloud telemetry
- Executive battery monitoring dashboard
- Fault counting and recovery management

## Integrated Internship Tasks

### 1. Adaptive Multi-Cell Battery Intelligence Engine
Performs real-time battery measurements and calculates cell voltage, pack voltage, average voltage, imbalance percentage, weakest cell, strongest cell, and battery health.

### 2. Event-Driven Safety Protection Kernel
Detects abnormal battery conditions and controls the relay, buzzer, and LEDs using non-blocking event-driven logic.

### 3. Intelligent Embedded HMI & Diagnostic Interface
Provides local battery measurements, analytics, system state, and fault information through a 16x2 I2C LCD.

### 4. Fault-Tolerant Embedded Runtime System
Manages system states, fault conditions, recovery logic, fault counting, and shutdown behavior.

### 5. Intelligent Cloud Telemetry Architecture
Transfers important battery and diagnostic information to Blynk through Wi-Fi with reconnection handling.

### 6. Executive Battery Intelligence Dashboard
Provides remote monitoring of battery measurements, health, system state, faults, relay status, risk level, and recommendations.

## Hardware

- ESP32
- 4 Potentiometers for simulated cell inputs
- 16x2 I2C LCD
- Red LED
- Yellow LED
- Green LED
- Buzzer
- Relay

## Software

- Embedded C/C++
- ESP32
- Wokwi
- Blynk IoT
- Arduino libraries

## Pin Configuration

| Component | ESP32 Pin |
|---|---|
| Cell 1 | GPIO 34 |
| Cell 2 | GPIO 35 |
| Cell 3 | GPIO 32 |
| Cell 4 | GPIO 33 |
| Red LED | GPIO 2 |
| Green LED | GPIO 4 |
| Yellow LED | GPIO 5 |
| Buzzer | GPIO 18 |
| Relay | GPIO 19 |

## System States

| State | Description |
|---|---|
| NORMAL | Battery operating normally |
| DEGRADED | Battery imbalance detected |
| FAILSAFE | Safety-critical battery fault |
| SHUTDOWN | Severe hardware or system fault |

## Fault Conditions Tested

1. Healthy Battery
2. Minor Imbalance
3. Critical Imbalance
4. Weak Cell
5. Overvoltage
6. Rapid Voltage Change
7. Sensor Fault
8. Frozen ADC
9. Relay Mismatch

All nine conditions were tested using Wokwi and verified through the Blynk dashboard.

## Project Links

### Wokwi Simulation
 https://wokwi.com/projects/473061276088437761

### Blynk Dashboard
https://blynk.cloud/dashboard/706483/global/devices/215401/organization/706483/devices/2245426/dashboard



## Author

**Sudanapalli Srinivas**

B.Tech. – Electronics and Communication Engineering

Vignan's Foundation for Science, Technology and Research (VFSTR)

## Project Status

Completed and validated through Wokwi simulation and Blynk IoT monitoring.
