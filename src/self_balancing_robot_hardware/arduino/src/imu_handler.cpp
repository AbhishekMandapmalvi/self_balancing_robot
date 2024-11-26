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
    bno.setMode(Adafruit_BNO055::OPERATION_MODE_IMUPLUS);
    
    isInitialized = true;
    return true;
}

bool IMUHandler::update() {
    if (!isInitialized) return false;
    
    unsigned long currentTime = millis();
    if (currentTime - lastUpdateTime < UPDATE_INTERVAL) {
        return true; // Not time to update yet
    }
    
    // Get Euler angles
    sensors_event_t orientationData;
    bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
    data.roll = orientationData.orientation.z;  // x-axis rotation
    data.pitch = orientationData.orientation.y; // y-axis rotation
    data.yaw = orientationData.orientation.x;   // z-axis rotation
    
    // Get angular velocities
    sensors_event_t angVelData;
    bno.getEvent(&angVelData, Adafruit_BNO055::VECTOR_GYROSCOPE);
    data.gyroX = angVelData.gyro.x;
    data.gyroY = angVelData.gyro.y;
    data.gyroZ = angVelData.gyro.z;
    
    // Get linear accelerations
    sensors_event_t linearAccelData;
    bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
    data.linearAccelX = linearAccelData.acceleration.x;
    data.linearAccelY = linearAccelData.acceleration.y;
    data.linearAccelZ = linearAccelData.acceleration.z;
    
    // Get calibration status
    bno.getCalibration(&data.systemCal, 
                       &data.gyroCal, 
                       &data.accelCal, 
                       &data.magCal);
    
    lastUpdateTime = currentTime;
    return true;
}

bool IMUHandler::isCalibrated() const {
    return (data.systemCal == 3 && 
            data.gyroCal == 3 && 
            data.accelCal == 3);
}