#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include <Arduino.h>
#include <stdint.h>
#include <Wire.h>
#include "pins_config.h"

class MotorController {
private:
    // Current state
    int leftSpeed;
    int rightSpeed;
    bool isEnabled;
    
public:
    MotorController();
    void init();
    
    // Control functions
    void setMotorSpeeds(int leftSpeed, int rightSpeed);
    void brake();
    
    // Getters
    int getLeftSpeed() const { return leftSpeed; }
    int getRightSpeed() const { return rightSpeed; }
};

#endif