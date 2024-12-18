#include <Arduino.h>
#include "motor_controller.h"
#include "encoder_handler.h"
#include "imu_handler.h"

class BalanceCar {
public:
    BalanceCar(MotorController& motors, EncoderHandler& encoders, IMUHandler& imu_handler);
    void update();
    void setTargetSpeed(int speed) { setting_car_speed = speed; }
    void setTurnSpeed(int turn_speed) { setting_turn_speed = turn_speed; }
    void setTurnAngle(float angle) { setting_turn_angle = angle; }
    bool isStable() const { return is_stable; }
    bool canRotate() const { return can_rotate; }
    float getCurrentAngle() const { return current_angle; }
    float angleWrap(float angle);

private:
    // References to external components
    MotorController& motors;
    EncoderHandler& encoders;
    IMUHandler& imu_handler;

    // PID Parameters
    double kp_balance = 25, kd_balance = 0.950;
    double kp_speed = -18, ki_speed = -0.07, kd_speed = -0.08;
    double kp_turn = 1.0, ki_turn = 0.001, kd_turn = 0.001;

    // Calibration parameters
    double angle_zero = 5.20;
    double angular_velocity_zero = 0;

    // Control variables
    double speed_control_output = 0;
    double rotation_control_output = 0;
    double speed_filter = 0;
    int speed_control_period_count = 0;
    double car_speed_integeral = 0;
    double speed_filter_old = 0;
    float setting_car_speed = 0;
    float setting_turn_speed = 0;
    float setting_turn_angle = 0;
    double pwm_left = 0;
    double pwm_right = 0;
    float current_angle = 0;
    float rotation_error = 0;

    // Balance wait parameters
    unsigned long stable_start_time = 0;
    bool is_stable = false;
    bool can_rotate = false;
    static const float STABLE_ANGLE_RANGE = 5.0;
    static const unsigned long STABLE_TIME_REQUIRED = 3000;

    // Balance angle limits
    static const float balance_angle_min = -22;
    static const float balance_angle_max = 22;

    // Timing and error tracking
    unsigned long last_time = 0;
    double last_speed_error = 0;
    double speed_error_rate = 0;
    double turn_integral = 0;
    double last_rotation_error = 0;
};