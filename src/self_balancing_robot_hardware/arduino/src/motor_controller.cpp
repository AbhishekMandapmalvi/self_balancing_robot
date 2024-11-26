#include "motor_controller.h"

MotorController::MotorController() : leftSpeed(0), rightSpeed(0) {}

void MotorController::init() {
    digitalWrite(MOTOR_STANDBY, HIGH);  // Always enable the driver
}

void MotorController::setMotorSpeeds(int left, int right) {
    // Constrain speeds
    left = constrain(left, -255, 255);
    right = constrain(right, -255, 255);
    
    // Left motor
    digitalWrite(MOTOR_LEFT_DIR, left >= 0 ? HIGH : LOW);
    analogWrite(MOTOR_LEFT_PWM, abs(left));
    leftSpeed = left;
    
    // Right motor
    digitalWrite(MOTOR_RIGHT_DIR, right >= 0 ? HIGH : LOW);
    analogWrite(MOTOR_RIGHT_PWM, abs(right));
    rightSpeed = right;
}

void MotorController::brake() {
    setMotorSpeeds(0, 0);
}