#ifndef IMU_HANDLER_H
#define IMU_HANDLER_H

#include <Arduino.h>
#include <stdint.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <EEPROM.h>
#include "pins_config.h"
#include "Rgb.h"

// Struct to store IMU data
struct IMUData {
    // Euler angles (degrees)
    float roll;
    float pitch;
    float yaw;
    // Motion data
    float gyroX, gyroY, gyroZ;
    float accelX, accelY, accelZ;
    // Quaternion representation of orientation
    float quatW, quatX, quatY, quatZ;
    // Calibration status
    uint8_t systemCal, gyroCal, accelCal, magCal;
};

// Class to handle IMU operations using Adafruit BNO055 sensor
class IMUHandler {
private:
    Adafruit_BNO055 bno; // BNO055 sensor object
    IMUData data; // Struct to store IMU data
    bool isInitialized; // Flag to indicate if the IMU is initialized successfully
    unsigned long lastUpdateTime; // Timestamp of the last update
    const unsigned long UPDATE_INTERVAL = 10; // 100Hz update rate
public:
    // Constructor: Initializes the IMUHandler object
    IMUHandler();
    
    // Initializes the BNO055 sensor and returns true if successful
    bool init();
    
    // Updates the IMU data at a fixed interval and returns true if successful
    bool update();
    
    // Returns the latest IMU data as an `IMUData` struct
    IMUData getData() const { return data; }
    
    // Checks if the sensor is fully calibrated (all components calibrated)
    bool isCalibrated() const;
    
    // Prints the calibration status of all components to the serial monitor
    void printCalibrationStatus();
    
    // Getter methods for accelerometer data (X, Y, Z axes)
    float getAccelX() const { return data.accelX; }
    float getAccelY() const { return data.accelY; }
    float getAccelZ() const { return data.accelZ; }
    
    // Getter methods for gyroscope data (angular velocity on X, Y, Z axes)
    float getGyroX() const { return data.gyroX; }
    float getGyroY() const { return data.gyroY; }
    float getGyroZ() const { return data.gyroZ; }
    
    // Getter methods for Euler angles (yaw, pitch, roll)
    float getYaw() const { return data.yaw; }
    float getPitch() const { return data.pitch; }
    float getRoll() const { return data.roll; }
    
    // Getter methods for quaternion components (W, X, Y, Z)
    float getquatW() const { return data.quatW; }
    float getquatX() const { return data.quatX; }
    float getquatY() const { return data.quatY; }
    float getquatZ() const { return data.quatZ; }
};

#endif
