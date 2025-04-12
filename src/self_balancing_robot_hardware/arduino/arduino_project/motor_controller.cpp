#include <Arduino.h>
#include <stdint.h>
#include "motor_controller.h"

MotorController::MotorController() : leftSpeed(0), rightSpeed(0) {}

void MotorController::init() {
    digitalWrite(MOTOR_STANDBY, HIGH);  // Always enable the driver
}

void MotorController::setLeftMotorSpeed(int16_t speed) {
    // Constrain speeds
    speed = constrain(speed, -255, 255);
    
    // Left motor
    digitalWrite(MOTOR_LEFT_DIR, speed >= 0 ? HIGH : LOW);
    analogWrite(MOTOR_LEFT_PWM, abs(speed));
}

void MotorController::setRightMotorSpeed(int16_t speed) {
    // Constrain speeds
    speed = constrain(speed, -255, 255);
    
    // Left motor
    digitalWrite(MOTOR_RIGHT_DIR, speed >= 0 ? HIGH : LOW);
    analogWrite(MOTOR_RIGHT_PWM, abs(speed));
}

void MotorController::brake() {
    setLeftMotorSpeed(0);
    setRightMotorSpeed(0);
}
