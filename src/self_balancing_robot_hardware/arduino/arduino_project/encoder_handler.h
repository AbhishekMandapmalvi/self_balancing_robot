#ifndef ENCODER_HANDLER_H
#define ENCODER_HANDLER_H

#include <Arduino.h>
#include <stdint.h>
#include <Wire.h>
#include "pins_config.h"
#include "imu_handler.h"

class EncoderHandler {
private:
    // References to external components
    IMUHandler& imu_handler;

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

    // Variables to track pose
    float x = 0.0, y = 0.0, theta = 0.0;
    void updatePose(float deltaLeft, float deltaRight, float deltaTime);

public:
    EncoderHandler();
    void init();
    void update();
    
    // Reset functions
    void resetCounts();
    void resetDistances();
    void resetPose();
    
    // Getters
    long getLeftCount() const { return leftCount; }
    long getRightCount() const { return rightCount; }
    float getLeftVelocity() const { return leftVelocity; }
    float getRightVelocity() const { return rightVelocity; }
    float getLeftDistance() const { return leftDistance; }
    float getRightDistance() const { return rightDistance; }

    // Getters for pose data
    float getX() const { return x; }
    float getY() const { return y; }
    float getTheta() const { return theta; }
    
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
