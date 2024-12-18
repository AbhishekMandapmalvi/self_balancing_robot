#include <Arduino.h>
#include <stdint.h>
#include "imu_handler.h"
#define BNO055_SYS_TRIGGER_ADDR 0x3F

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
    
    // Set to NDOF_FMC_OFF mode directly
    bno.setMode(OPERATION_MODE_NDOF_FMC_OFF);
    bno.setExtCrystalUse(true);
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
    data.pitch = event.orientation.z;
    data.roll = event.orientation.y;

    // Get motion data
    imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
    
    data.gyroX = gyro.x();
    data.gyroY = gyro.y();
    data.gyroZ = gyro.z();
    
    data.accelX = accel.x();
    data.accelY = accel.y();
    data.accelZ = accel.z();

    // Get quaternion data
    imu::Quaternion quat = bno.getQuat();
    data.quatW = quat.w();
    data.quatX = quat.x();
    data.quatY = quat.y();
    data.quatZ = quat.z();

    float norm = sqrt(data.quatW * data.quatW + 
                  data.quatX * data.quatX + 
                  data.quatY * data.quatY + 
                  data.quatZ * data.quatZ);

    data.quatW /= norm;
    data.quatX /= norm;
    data.quatY /= norm;
    data.quatZ /= norm;
   
    lastUpdateTime = currentTime;
    return true;
}

void IMUHandler::printCalibrationStatus() {
    if (!isInitialized) {
        Serial.println("IMU not initialized!");
        return;
    }
    
    // Update calibration status
    bno.getCalibration(&data.systemCal, &data.gyroCal, 
                       &data.accelCal, &data.magCal);
    
    //Serial.print("System: ");
    //Serial.print(data.systemCal);
    //Serial.print("/3 | Gyro: ");
    //Serial.print(data.gyroCal);
    //Serial.print("/3 | Accel: ");
    //Serial.print(data.accelCal);
    //Serial.print("/3 | Mag: ");
    //Serial.print(data.magCal);
    //Serial.println("/3");
    
    // Blink red LEDs when system calibration is 0
    if (data.systemCal == 0) {
        rgb.flashRedColor();
    } else {
        rgb.lightOff();
    }
}
bool IMUHandler::isCalibrated() const {
    return (data.systemCal == 3 && data.gyroCal == 3 && 
            data.accelCal == 3 && data.magCal == 3);
}
