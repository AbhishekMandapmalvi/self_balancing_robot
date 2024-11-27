#include <Arduino.h>
#include <stdint.h>
#include "imu_handler.h"

IMUHandler::IMUHandler() : 
    bno(55, BNO055_I2C_ADDR),
    isInitialized(false),
    lastUpdateTime(0) {
    memset(&data, 0, sizeof(IMUData));
}

bool IMUHandler::init() {
    if (!bno.begin()) {
        return false;
    }
    
    delay(1000); // Allow time for sensor to stabilize
    
    // Use external crystal for better accuracy
    bno.setExtCrystalUse(true);
    
    // Set to IMU mode (fusion of accelerometer and gyroscope)
    bno.setMode(OPERATION_MODE_NDOF);
    
    isInitialized = true;
    return true;
}

bool IMUHandler::update() {
    if (!isInitialized) return false;
    
    unsigned long currentTime = millis();
    if (currentTime - lastUpdateTime < UPDATE_INTERVAL) {
        return true; // Not time to update yet
    }
    
    // Get orientation data
    sensors_event_t event;
    bno.getEvent(&event);
    data.yaw = event.orientation.x;
    data.roll = event.orientation.z;
    data.pitch = event.orientation.y;

    // Get quaternion data
    imu::Quaternion quat = bno.getQuat();
    data.qw = quat.w();
    data.qx = quat.x();
    data.qy = quat.y();
    data.qz = quat.z();

    // Get motion data
    imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
    
    data.gyroX = gyro.x();
    data.gyroY = gyro.y();
    data.gyroZ = gyro.z();
    
    data.accelX = accel.x();
    data.accelY = accel.y();
    data.accelZ = accel.z();

    // Get calibration status
    bno.getCalibration(&data.systemCal, &data.gyroCal, 
                       &data.accelCal, &data.magCal);
    
    lastUpdateTime = currentTime;
    return true;
}

bool IMUHandler::isCalibrated() const {
    return (data.systemCal == 3 && data.gyroCal == 3 && 
            data.accelCal == 3 && data.magCal == 3);
}