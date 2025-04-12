# Self-Balancing Robot with ROS Integration

Welcome to the **Self-Balancing Robot** project repository! This ongoing project involves building a self-balancing robot equipped with cascaded PID controllers for stability, real-time control using Arduino Mega, and advanced ROS-based tasks managed by a Raspberry Pi. The robot is designed to balance itself using IMU data from the BNO055 sensor and navigate autonomously using camera perception, path planning, and navigation.

---

## Features

### 1. Self-Balancing Mechanism
- Implemented **cascaded PID controllers** to achieve precise balance control.
- Real-time control tasks are handled by an **Arduino Mega** to ensure no delays in balancing.
- IMU readings (pitch, yaw, roll, and motion data) are provided by the **BNO055 sensor**.

### 2. ROS Integration
- A **Raspberry Pi** is used to manage high-level tasks such as:
  - **Camera perception** for obstacle detection and environment mapping.
  - **Path planning** for autonomous navigation.
  - **Navigation** using ROS topics like `cmd_vel` and `odom`.

### 3. Modular Design
- Separation of low-level control (Arduino Mega) and high-level tasks (Raspberry Pi) ensures efficient operation.
- Communication between Arduino and Raspberry Pi is handled via ROS nodes.

---

## Hardware Components

### Microcontrollers
- **Arduino Mega 2560**: Handles real-time control tasks such as motor control, PID computation, and IMU data processing.
- **Raspberry Pi (Model 3/4)**: Manages ROS-based tasks including camera processing and navigation.

### Sensors
- **Adafruit BNO055 IMU Sensor**: Provides accurate orientation (pitch, roll, yaw), gyroscope, accelerometer, and quaternion data for balancing.

### Actuators
- **DC Motors with Encoders**: Controlled via PWM signals from Arduino for precise movement.

### Other Components
- **RGB LEDs**: Indicate system status such as calibration levels.
- **Ultrasonic Sensors**: Optional for obstacle detection.
- **Camera Module**: Used for vision-based navigation.

---

## Software Architecture

### Arduino Mega
The Arduino handles:
- Real-time PID control for balancing the robot.
- Reading IMU data from the BNO055 sensor.
- Motor control using PWM signals.
- Publishing encoder data (`odom`) to ROS topics.

### Raspberry Pi with ROS
The Raspberry Pi manages:
- Subscribing to `cmd_vel` topic for velocity commands.
- Camera perception for obstacle detection and mapping.
- Path planning algorithms to navigate autonomously.
- Publishing odometry data (`odom`) using encoder readings and IMU data.

---
