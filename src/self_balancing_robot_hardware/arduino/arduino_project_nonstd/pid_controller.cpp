#include "pid_controller.h"

BalanceCar::BalanceCar(MotorController& m, EncoderHandler& e, IMUHandler& i)
    : motors(m), encoders(e), imu_handler(i) {
}

float BalanceCar::angleWrap(float angle) {
    while (angle > 180) {
        angle -= 360;
    }
    while (angle < -180) {
        angle += 360;
    }
    return angle;
}

void BalanceCar::update() {
    unsigned long current_time = millis();
    double delta_time = (current_time - last_time) / 1000.0;
    
    imu_handler.update();
    encoders.update();
    
    IMUData imu_data = imu_handler.getData();
    current_angle = imu_data.pitch;
    float current_gyro = imu_data.gyroX;
    
    float angle_error = abs(current_angle - angle_zero);
    if (angle_error <= STABLE_ANGLE_RANGE) {
        if (!is_stable) {
            is_stable = true;
            stable_start_time = current_time;
        }
        else if (current_time - stable_start_time >= STABLE_TIME_REQUIRED) {
            can_rotate = true;
        }
    } else {
        is_stable = false;
        can_rotate = false;
        stable_start_time = current_time;
    }

    double balance_control_output = kp_balance * (current_angle - angle_zero)
                                  + kd_balance * (-current_gyro + angular_velocity_zero);

    speed_control_period_count++;
    if (speed_control_period_count >= 8) {
        speed_control_period_count = 0;
        
        double car_speed = (encoders.getLeftVelocity() + encoders.getRightVelocity()) * 0.5;
        speed_filter = speed_filter_old * 0.6 + car_speed * 0.4;
        speed_filter_old = speed_filter;
        
        double speed_error = speed_filter - setting_car_speed;
        speed_error_rate = (speed_error - last_speed_error) / (8 * delta_time);
        last_speed_error = speed_error;
        
        car_speed_integeral += speed_error;
        car_speed_integeral = constrain(car_speed_integeral, -200, 200);

        speed_control_output = -kp_speed * speed_filter - ki_speed * car_speed_integeral - kd_speed * speed_error_rate;
        
        if (can_rotate) {
            // Convert yaw to range -180 to 180
            float current_yaw = imu_data.yaw;
            if (current_yaw > 180) {
                current_yaw -= 360;
            }
            
            // Calculate shortest angle difference
            rotation_error = angleWrap(setting_turn_angle - current_yaw);
            turn_integral += rotation_error * 8 * delta_time;
            turn_integral = constrain(turn_integral, -1000, 1000);
            
            float rotation_error_rate = (rotation_error - last_rotation_error) / (8 * delta_time);
            last_rotation_error = rotation_error;
            
            rotation_control_output = kp_turn * rotation_error + 
                                    ki_turn * turn_integral + 
                                    kd_turn * rotation_error_rate;
        } else {
            rotation_control_output = 0;
            turn_integral = 0;
            last_rotation_error = 0;
        }
    }

    pwm_left = balance_control_output - speed_control_output - rotation_control_output;
    pwm_right = balance_control_output - speed_control_output + rotation_control_output;

    if (abs(pwm_left) < 20) pwm_left = 0;
    if (abs(pwm_right) < 20) pwm_right = 0;

    pwm_left = constrain(pwm_left, -255, 255);
    pwm_right = constrain(pwm_right, -255, 255);

    if (current_angle < balance_angle_min || balance_angle_max < current_angle) {
        motors.brake();
        return;
    }
    
    last_time = current_time;
    
    motors.setLeftMotorSpeed(pwm_left);
    motors.setRightMotorSpeed(pwm_right);
}
