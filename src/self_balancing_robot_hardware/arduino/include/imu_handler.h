#ifndef IMU_HANDLER_H
#define IMU_HANDLER_H

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include "pins_config.h"

struct IMUData {
    // Euler angles (degrees)
    float roll;
    float pitch;
    float yaw;
    // Angular velocities (rad/s)
    float gyroX;
    float gyroY;
    float gyroZ;
    // Linear accelerations (m/s^2)
    float accelX;
    float accelY;
    float accelZ;
    // Calibration status
    uint8_t systemCal;
    uint8_t gyroCal;
    uint8_t accelCal;
    uint8_t magCal;
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
    bool isCalibrated() const;
    IMUData getData() const { return data; }

    // Individual getters
    float getRoll() const { return data.roll; }
    float getPitch() const { return data.pitch; }
    float getYaw() const { return data.yaw; }
    float getGyroX() const { return data.gyroX; }
    float getGyroY() const { return data.gyroY; }
    float getGyroZ() const { return data.gyroZ; }
    float getLinearAccelX() const { return data.linearAccelX; }
    float getLinearAccelY() const { return data.linearAccelY; }
    float getLinearAccelZ() const { return data.linearAccelZ; }
};

#endif
