#include "pid_controller.h"

// Constructor for the BalanceCar class, initializes motor, encoder, and IMU handlers
BalanceCar::BalanceCar(MotorController& m, EncoderHandler& e, IMUHandler& i)
    : motors(m), encoders(e), imu_handler(i) {
}

// Helper function to wrap an angle to the range [-180, 180] degrees
float BalanceCar::angleWrap(float angle) {
    while (angle > 180) {
        angle -= 360;
    }
    while (angle < -180) {
        angle += 360;
    }
    return angle;
}

// Main update function for the balance car's control loop
void BalanceCar::update() {
    // Get the current time and calculate time difference since the last update
    unsigned long current_time = millis();
    double delta_time = (current_time - last_time) / 1000.0;

    // Update sensor data from IMU and encoders
    imu_handler.update();
    encoders.update();

    // Retrieve IMU data (pitch and gyro readings)
    IMUData imu_data = imu_handler.getData();
    current_angle = imu_data.pitch;
    float current_gyro = imu_data.gyroX;

    // Check if the car is within a stable angle range
    float angle_error = abs(current_angle - angle_zero);
    if (angle_error <= STABLE_ANGLE_RANGE) {
        // If the car just became stable, record the start time
        if (!is_stable) {
            is_stable = true;
            stable_start_time = current_time;
        }
        else if (current_time - stable_start_time >= STABLE_TIME_REQUIRED) {
            // If the car has been stable for a required amount of time, allow rotation
            can_rotate = true;
        }
    } else {
        // If the car is not stable, reset stability flags and timers
        is_stable = false;
        can_rotate = false;
        stable_start_time = current_time;
    }
    
    // Calculate balance control output using proportional-derivative (PD) control
    double balance_control_output = kp_balance * (current_angle - angle_zero)
                                  + kd_balance * (-current_gyro + angular_velocity_zero);

    // Perform speed control calculations periodically (every 8 updates)
    speed_control_period_count++;
    if (speed_control_period_count >= 8) {
        speed_control_period_count = 0;

        // Calculate average car speed from encoder velocities
        double car_speed = (encoders.getLeftVelocity() + encoders.getRightVelocity()) * 0.5;
        // Apply a low-pass filter to smooth out speed measurements
        speed_filter = speed_filter_old * 0.6 + car_speed * 0.4;
        speed_filter_old = speed_filter;

        // Calculate speed error and its derivative for PID control
        double speed_error = speed_filter - setting_car_speed;
        speed_error_rate = (speed_error - last_speed_error) / (8 * delta_time);
        last_speed_error = speed_error;

        // Accumulate integral of speed error and constrain it to prevent windup
        car_speed_integeral += speed_error;
        car_speed_integeral = constrain(car_speed_integeral, -200, 200);

        // Calculate speed control output using PID formula
        speed_control_output = -kp_speed * speed_filter - ki_speed * car_speed_integeral - kd_speed * speed_error_rate;
        
        if (can_rotate) {
            // If rotation is allowed, calculate rotation control output
            // Convert yaw to range -180 to 180
            float current_yaw = imu_data.yaw;
            if (current_yaw > 180) {
                current_yaw -= 360;
            }
            
            // Calculate shortest angle difference for rotation error
            rotation_error = angleWrap(setting_turn_angle - current_yaw);
            turn_integral += rotation_error * 8 * delta_time;
            turn_integral = constrain(turn_integral, -1000, 1000);

            // Accumulate integral of rotation error and constrain it
            float rotation_error_rate = (rotation_error - last_rotation_error) / (8 * delta_time);
            last_rotation_error = rotation_error;
            
            // Calculate rotation control output using PID formula
            rotation_control_output = kp_turn * rotation_error + 
                                    ki_turn * turn_integral + 
                                    kd_turn * rotation_error_rate;
        } else {
            // Reset rotation-related variables if rotation is not allowed
            rotation_control_output = 0;
            turn_integral = 0;
            last_rotation_error = 0;
        }
    }

    // Combine balance, speed, and rotation control outputs to calculate motor PWM signals
    pwm_left = balance_control_output - speed_control_output - rotation_control_output;
    pwm_right = balance_control_output - speed_control_output + rotation_control_output;

    // Apply a dead zone to small PWM values to avoid unnecessary motor activation
    if (abs(pwm_left) < 20) pwm_left = 0;
    if (abs(pwm_right) < 20) pwm_right = 0;

    // Constrain PWM values to allowable range [-255, 255]
    pwm_left = constrain(pwm_left, -255, 255);
    pwm_right = constrain(pwm_right, -255, 255);

    // If the car's pitch angle exceeds safe limits, stop the motors immediately
    if (current_angle < balance_angle_min || balance_angle_max < current_angle) {
        motors.brake();
        return;
    }

    // Update the last time for the next iteration of the loop
    last_time = current_time;
    
    // Set motor speeds using calculated PWM values
    motors.setLeftMotorSpeed(pwm_left);
    motors.setRightMotorSpeed(pwm_right);
}
