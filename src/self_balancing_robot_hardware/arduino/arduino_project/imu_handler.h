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

struct IMUData {
    // Euler angles (degrees)
    float roll;
    float pitch;
    float yaw;
    // Motion data
    float gyroX, gyroY, gyroZ;
    float accelX, accelY, accelZ;
    float quatW, quatX, quatY, quatZ;
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
    void printCalibrationStatus();
    float getAccelX() const { return data.accelX; }
    float getAccelY() const { return data.accelY; }
    float getAccelZ() const { return data.accelZ; }
    float getGyroX() const { return data.gyroX; }
    float getGyroY() const { return data.gyroY; }
    float getGyroZ() const { return data.gyroZ; }
    float getYaw() const { return data.yaw; }
    float getPitch() const { return data.pitch; }
    float getRoll() const { return data.roll; }
    float getquatW() const { return data.quatW; }
    float getquatX() const { return data.quatX; }
    float getquatY() const { return data.quatY; }
    float getquatZ() const { return data.quatZ; }
};

#endif
