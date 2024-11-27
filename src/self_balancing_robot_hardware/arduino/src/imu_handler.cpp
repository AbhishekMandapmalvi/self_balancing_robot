#ifndef IMU_HANDLER_H
#define IMU_HANDLER_H

#include <Arduino.h>
#include <stdint.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include "pins_config.h"

struct IMUData {
    // Euler angles (degrees)
    float roll;
    float pitch;
    float yaw;
    // Quaternion data
    float qw, qx, qy, qz;
    // Motion data
    float gyroX, gyroY, gyroZ;
    float accelX, accelY, accelZ;
    // Calibration status
    uint8_t systemCal, gyroCal, accelCal, magCal;
};

class IMUHandler {
private:
    Adafruit_BNO055 bno;
    IMUData data;
    bool isInitialized;
    unsigned long lastUpdateTime;
    const unsigned long UPDATE_INTERVAL = 10; // 100Hz update rate
    
public:
    IMUHandler();
    bool init();
    bool update();
    IMUData getData() const { return data; }
    bool isCalibrated() const;

};

#endif