#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include <Arduino.h>
#include <stdint.h>
#include <Wire.h>
#include "pins_config.h"

class MotorController {
private:
    // Current state
    int16_t leftSpeed;
    int16_t rightSpeed;
    bool isEnabled;
    
public:
    MotorController();
    void init();
    
    // Control functions
    void setLeftMotorSpeed(int16_t speed);
    void setRightMotorSpeed(int16_t speed);
    void brake();
    
    // Getters
    int16_t getLeftSpeed() const { return leftSpeed; }
    int16_t getRightSpeed() const { return rightSpeed; }
};

#endif