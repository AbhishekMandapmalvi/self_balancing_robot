#ifndef ENCODER_HANDLER_H
#define ENCODER_HANDLER_H

#include "pins_config.h"

class EncoderHandler {
private:
    // Encoder counts
    volatile long leftCount;
    volatile long rightCount;
    volatile long lastLeftCount;
    volatile long lastRightCount;
    
    // Right encoder state tracking
    uint8_t rightEncoderState;
    static const int8_t stateTable[16];  // State transition table
    
    // Timing
    unsigned long lastUpdateTime;
    const unsigned long UPDATE_INTERVAL = 50; // ms
    
    // Constants
    const float TICKS_PER_REV = 26.0;    // Encoder ticks per revolution
    const float GEAR_RATIO = 30.0;      // Motor gear ratio
    const float WHEEL_DIAMETER = 0.065;   // meters
    
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
    void handleLeftEncoder();
    void checkRightEncoder();

private:
    // Helper function to validate and process right encoder state
    int8_t processRightEncoderState(uint8_t newState);
};

// Interrupt service routine declarations
void leftEncoderISR();

#endif