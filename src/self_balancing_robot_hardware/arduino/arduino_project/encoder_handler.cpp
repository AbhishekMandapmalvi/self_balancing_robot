#include <Arduino.h>
#include <stdint.h>
#include "encoder_handler.h"

// Global instance for ISR access
EncoderHandler* encoderInstance = nullptr;

EncoderHandler::EncoderHandler() : 
    imu_handler(imu_handler),
    leftCount(0), rightCount(0),
    lastLeftCount(0), lastRightCount(0),
    lastUpdateTime(0),
    leftVelocity(0), rightVelocity(0),
    leftDistance(0), rightDistance(0),
    x(0.0), y(0.0), theta(0.0) {
    encoderInstance = this;
}

void EncoderHandler::init() {
    // Attach interrupts for both encoders
    attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_A), leftEncoderA_ISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_B), leftEncoderB_ISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_A), rightEncoderA_ISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_B), rightEncoderB_ISR, CHANGE);
    
    lastUpdateTime = millis();
}

void EncoderHandler::update() {
    unsigned long currentTime = millis();
    float deltaTime = (currentTime - lastUpdateTime) / 1000.0;

    if (deltaTime >= UPDATE_INTERVAL / 1000.0) {
        long deltaLeft = leftCount - lastLeftCount;
        long deltaRight = rightCount - lastRightCount;

        // Calculate velocities with low-pass filtering
        float newLeftVelocity = 2 * PI * (deltaLeft) / 
                               (TICKS_PER_REV * GEAR_RATIO * deltaTime);
        float newRightVelocity = 2 * PI * (deltaRight) / 
                                (TICKS_PER_REV * GEAR_RATIO * deltaTime);

        // Apply low-pass filter (alpha = 0.7 is just an example)
        const float alpha = 0.7;
        leftVelocity = alpha * newLeftVelocity + (1 - alpha) * leftVelocity;
        rightVelocity = alpha * newRightVelocity + (1 - alpha) * rightVelocity;

        // Update distances (meters)
        float leftDelta = deltaLeft * PI * WHEEL_DIAMETER / 
                         (TICKS_PER_REV * GEAR_RATIO);
        float rightDelta = deltaRight * PI * WHEEL_DIAMETER / 
                          (TICKS_PER_REV * GEAR_RATIO);

        leftDistance += leftDelta;
        rightDistance += rightDelta;

        // Compute linear velocity of the robot
        float v = (leftDelta + rightDelta) / 2.0;  // Linear velocity (m/s)

        // Update orientation using IMU data
        theta = imu_handler.getYaw() * DEG_TO_RAD; // Get yaw from IMU in radians

        // Normalize theta to the range [-pi, pi]
        if (theta > PI) theta -= 2 * PI;
        if (theta < -PI) theta += 2 * PI;

        // Update position (x, y)
        x += v * cos(theta) * deltaTime;
        y += v * sin(theta) * deltaTime;

        // Save current counts and time for next update
        lastLeftCount = leftCount;
        lastRightCount = rightCount;
        lastUpdateTime = currentTime;
    }
}

void EncoderHandler::resetCounts() {
    noInterrupts();
    leftCount = 0;
    rightCount = 0;
    lastLeftCount = 0;
    lastRightCount = 0;
    interrupts();
}

void EncoderHandler::resetDistances() {
    noInterrupts();
    leftDistance = 0;
    rightDistance = 0;
    interrupts();
}

void EncoderHandler::resetPose() {
     noInterrupts();
     x = 0.0;
     y = 0.0;
     theta = 0.0;
     interrupts();
}

// New encoder handling methods using state machine
void EncoderHandler::handleLeftEncoder(uint8_t pin) {
    static uint8_t oldAB = 0;
    static int8_t encTable[] = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};
    
    oldAB <<= 2;
    if (digitalRead(ENCODER_LEFT_A)) oldAB |= 0x02;
    if (digitalRead(ENCODER_LEFT_B)) oldAB |= 0x01;
    leftCount -= encTable[oldAB & 0x0F];  // Note the minus sign here
}

void EncoderHandler::handleRightEncoder(uint8_t pin) {
    static uint8_t oldAB = 0;
    static int8_t encTable[] = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};
    
    oldAB <<= 2;
    if (digitalRead(ENCODER_RIGHT_A)) oldAB |= 0x02;
    if (digitalRead(ENCODER_RIGHT_B)) oldAB |= 0x01;
    rightCount += encTable[oldAB & 0x0F];
}

// ISR functions
void leftEncoderA_ISR() {
    if (encoderInstance) {
        encoderInstance->handleLeftEncoder(ENCODER_LEFT_A);
    }
}

void leftEncoderB_ISR() {
    if (encoderInstance) {
        encoderInstance->handleLeftEncoder(ENCODER_LEFT_B);
    }
}

void rightEncoderA_ISR() {
    if (encoderInstance) {
        encoderInstance->handleRightEncoder(ENCODER_RIGHT_A);
    }
}

void rightEncoderB_ISR() {
    if (encoderInstance) {
        encoderInstance->handleRightEncoder(ENCODER_RIGHT_B);
    }
}
