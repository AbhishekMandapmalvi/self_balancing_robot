# Self-Balancing Robot with ROS Integration

Welcome to the **Self-Balancing Robot** project repository! This ongoing project involves building a self-balancing robot equipped with cascaded PID controllers for stability, real-time control using Arduino Mega, and advanced ROS-based tasks managed by a Raspberry Pi. The robot balances itself by fusing encoder and IMU data from the BNO055 sensor and navigates autonomously using camera perception, path planning, and navigation. **This project will eventually integrate with  [MPC-Project](https://github.com/AbhishekMandapmalvi/MPC-Project/blob/main/README.md)**, a companion repository implementing a unicycle kinematic model for dynamic obstacle avoidance, enabling advanced maneuvers in complex environments.


https://github.com/user-attachments/assets/2c6ad56e-2e7e-4330-a570-75c188bcdd77

---

## Features

### 1. Self-Balancing Mechanism
- Implemented **three cascaded PID controllers** to achieve precise balance control.
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
![HardwareArchitecture](https://github.com/user-attachments/assets/ec29643a-c434-4a69-9cc2-6997e8ace3db)

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

### Cascaded PID Control for Self-Balancing Robot  

This module explains the **three cascaded PID controllers** used in a self-balancing robot to maintain stability, regulate speed, and manage rotation. The system employs a hierarchical control architecture where each controller’s output influences the next, enabling precise balancing and motion control.  

#### Overview  
The robot relies on a layered control system:  
1. **Balance Controller**: Keeps the robot from falling.  
2. **Speed Controller**: Maintains steady movement.  
3. **Rotation Controller**: Enables smooth turns.  

#### Key Components  
- **Sensors**:  
  - **IMU** (measures tilt angle and rotation speed).  
  - **Wheel Encoders** (track speed).  
- **Motors**: Adjust wheel speed to balance and move.  

#### How It Works  

##### 1. Balance Controller ("Don’t Fall Over")  
- **Job**: Keep the robot upright.  
- **How**:  
  - Constantly checks the robot’s **tilt angle** (e.g., leaning forward/backward).  
  - If leaning forward, spins wheels backward to correct. If leaning backward, spins wheels forward.  
  - Uses **tilt speed** (from gyroscope) to predict and smooth wobbles.  

##### 2. Speed Controller ("Steady Speed")  
- **Job**: Maintain a set speed (e.g., move forward at 0.5 m/s).  
- **How**:  
  - Compares actual speed (from wheel encoders) to target speed.  
  - If too slow, gently tips the robot forward to speed up. If too fast, tips backward to slow down.  
  - Adjusts the **balance controller’s target angle** to create motion.  

##### 3. Rotation Controller ("Smooth Turn")  
- **Job**: Turn left/right without losing balance.  
- **How**:  
  - Measures the robot’s **direction** (using IMU’s yaw angle).  
  - To turn left, slows the left wheel and speeds up the right wheel (and vice versa).  
  - Ensures turns don’t disrupt balance.
  - 
#### How They Work Together  
1. **Balance is Priority**:  
   - The balance controller runs **50–100 times per second** to prevent falls.  
   - Speed and rotation controllers run slower (**~10 times per second**).  

2. **Example: Moving Forward**  
   - The speed controller tips the robot slightly forward.  
   - The balance controller detects the tilt and drives the wheels to "catch" the robot, creating forward motion.  

3. **Example: Turning While Moving**  
   - The rotation controller adds a small speed difference between wheels.  
   - The balance controller keeps the robot upright during the turn.  

#### Tuning Order  
1. **Balance First**: Tune to keep the robot stable.  
2. **Speed Next**: Adjust to maintain steady motion.  
3. **Rotation Last**: Fine-tune for smooth turns.
   
#### Safety Features  
- **Emergency Brake**: Stops motors if the tilt exceeds safe limits.  
- **Dead Zone**: Ignores tiny motor adjustments to prevent jitter.  

This cascaded system ensures the robot balances, moves, and turns smoothly—like riding a bike! 🚴♂️ 

### Raspberry Pi with ROS
The Raspberry Pi manages:
- Subscribing to `cmd_vel` topic for velocity commands.
- Camera perception for obstacle detection and mapping.
- Path planning algorithms to navigate autonomously.
- Publishing odometry data (`odom`) using encoder readings and IMU data.

---
