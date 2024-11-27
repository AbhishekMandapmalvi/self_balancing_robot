#include <Arduino.h>
#include <stdint.h>
#include "encoder_handler.h"

// Global instance for ISR access
EncoderHandler* encoderInstance = nullptr;

EncoderHandler::EncoderHandler() : 
    leftCount(0), rightCount(0),
    lastLeftCount(0), lastRightCount(0),
    lastRightState(0), 
    lastUpdateTime(0),
    leftVelocity(0), rightVelocity(0),
    leftDistance(0), rightDistance(0) {
    encoderInstance = this;
}

void EncoderHandler::init() {
    attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_A), leftEncoderISR, RISING);
    // Initialize right encoder state
    lastRightState = (digitalRead(ENCODER_RIGHT_A) << 1) | digitalRead(ENCODER_RIGHT_B);
        
    lastUpdateTime = millis();
}

void EncoderHandler::update() {
    // Check right encoder
    checkRightEncoder();

    unsigned long currentTime = millis();
    float deltaTime = (currentTime - lastUpdateTime) / 1000.0;
    
    if (deltaTime >= UPDATE_INTERVAL/1000.0) {
        long deltaLeft = leftCount - lastLeftCount;
        long deltaRight = rightCount - lastRightCount;        
        
        // Calculate velocities (rad/s)
        leftVelocity = 2 * PI * (deltaLeft) / 
                       (TICKS_PER_REV * GEAR_RATIO * deltaTime);
        rightVelocity = 2 * PI * (deltaRight) / 
                        (TICKS_PER_REV * GEAR_RATIO * deltaTime);
        
        // Update distances (meters)
        float leftDelta = deltaLeft * PI * WHEEL_DIAMETER / 
                         (TICKS_PER_REV * GEAR_RATIO);
        float rightDelta = deltaRight * PI * WHEEL_DIAMETER / 
                          (TICKS_PER_REV * GEAR_RATIO);
        
        leftDistance += leftDelta;
        rightDistance += rightDelta;
        
        lastLeftCount = leftCount;
        lastRightCount = rightCount;
        lastUpdateTime = currentTime;
    }
}

void EncoderHandler::resetCounts() {
    noInterrupts(); // Disable interrupts temporarily
    leftCount = 0;
    rightCount = 0;
    lastLeftCount = 0;
    lastRightCount = 0;
    interrupts(); // Re-enable interrupts
}

void EncoderHandler::resetDistances() {
    noInterrupts();
    leftDistance = 0;
    rightDistance = 0;
    interrupts(); // Re-enable interrupts
}

void EncoderHandler::handleLeftEncoder() {
    // Read the B channel to determine direction
    if (digitalRead(ENCODER_LEFT_B) == digitalRead(ENCODER_LEFT_A)) {
        leftCount++;
    } else {
        leftCount--;
    }
}

void EncoderHandler::checkRightEncoder() {
    // Current state table: {prev_state, current_state} -> direction
    static const int8_t stateTable[16] = {
        0,  // 0000 - no movement
        +1, // 0001 - clockwise
        -1, // 0010 - counter-clockwise
        0,  // 0011 - invalid
        -1, // 0100 - counter-clockwise
        0,  // 0101 - no movement
        0,  // 0110 - invalid
        +1, // 0111 - clockwise
        +1, // 1000 - clockwise
        0,  // 1001 - invalid
        0,  // 1010 - no movement
        -1, // 1011 - counter-clockwise
        0,  // 1100 - invalid
        -1, // 1101 - counter-clockwise
        +1, // 1110 - clockwise
        0   // 1111 - no movement
    };
    
    static uint8_t oldState = 0;
    
    // Read current state
    uint8_t state = (digitalRead(ENCODER_RIGHT_A) << 1) | digitalRead(ENCODER_RIGHT_B);
    uint8_t combined = (oldState << 2) | state;
    
    // Get direction from state table
    int8_t direction = stateTable[combined];
    
    // Update counter and state
    rightCount += direction;
    oldState = state;
}

// ISR functions
void leftEncoderISR() {
    if (encoderInstance) {
        encoderInstance->handleLeftEncoder();
    }
}