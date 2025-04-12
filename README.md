# Self-Balancing Robot with ROS Integration

Welcome to the **Self-Balancing Robot** project repository! This ongoing project involves building a self-balancing robot equipped with cascaded PID controllers for stability, real-time control using Arduino Mega, and advanced ROS-based tasks managed by a Raspberry Pi. The robot balances itself by fusing encoder and IMU data from the BNO055 sensor and navigates autonomously using camera perception, path planning, and navigation. **This project will eventually integrate with [ABC](https://github.com/your-username/ABC)**, a companion repository implementing a unicycle kinematic model for dynamic obstacle avoidance, enabling advanced maneuvers in complex environments.


https://github.com/user-attachments/assets/2c6ad56e-2e7e-4330-a570-75c188bcdd77

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
- **Raspberry Pi 4B 8GB**: Manages ROS-based tasks including camera processing and navigation.

### Sensors
- **Adafruit BNO055 IMU Sensor**: Provides accurate orientation (pitch, roll, yaw), gyroscope, accelerometer, and quaternion data for balancing.
- **Encoders**: Provides precise position and velocity.

### Actuators
- **DC Motors with Encoders**: Controlled via PWM signals from Arduino for precise movement.

### Other Components
- **RGB LEDs**: Indicate system status such as calibration levels.
- **Ultrasonic Sensors**: Optional for obstacle detection.
- **Camera Module**: 5MP OV5647 used for vision-based navigation.

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
