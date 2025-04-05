#ifndef ENCODER_HANDLER_H
#define ENCODER_HANDLER_H

#include <Arduino.h>
#include <stdint.h>
#include <Wire.h>
#include "pins_config.h"

class EncoderHandler {
private:
    // Encoder counts
    volatile long leftCount;
    volatile long rightCount;
    volatile long lastLeftCount;
    volatile long lastRightCount;
    
    // Timing
    unsigned long lastUpdateTime;
    const unsigned long UPDATE_INTERVAL = 10; // ms
    
    // Constants
    const float TICKS_PER_REV = 26.0;    // Encoder ticks per revolution
    const float GEAR_RATIO = 30.0;      // Motor gear ratio
    const float WHEEL_DIAMETER = 0.043;   //0.068 meters
    
    // Calculated values
    float leftVelocity;  // rad/s
    float rightVelocity; // rad/s
    float leftDistance;  // meters
    float rightDistance; // meters

public:
    EncoderHandler();
    void init();
    void update();
    
    // Reset functions
    void resetCounts();
    void resetDistances();
    
    // Getters
    long getLeftCount() const { return leftCount; }
    long getRightCount() const { return rightCount; }
    float getLeftVelocity() const { return leftVelocity; }
    float getRightVelocity() const { return rightVelocity; }
    float getLeftDistance() const { return leftDistance; }
    float getRightDistance() const { return rightDistance; }
    
    // Encoder handlers
    void handleLeftEncoder(uint8_t pin);
    void handleRightEncoder(uint8_t pin);
};

// Interrupt service routine declarations
void leftEncoderA_ISR();
void leftEncoderB_ISR();
void rightEncoderA_ISR();
void rightEncoderB_ISR();

#endif