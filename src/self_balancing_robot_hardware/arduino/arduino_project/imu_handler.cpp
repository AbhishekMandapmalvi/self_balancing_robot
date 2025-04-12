#include <Arduino.h>
#include <stdint.h>
#include "imu_handler.h"
#define BNO055_SYS_TRIGGER_ADDR 0x3F

// Constructor for IMUHandler
// Initializes the BNO055 sensor object, sets the initialization flag to false, 
// and clears the IMUData structure.
IMUHandler::IMUHandler() : 
    bno(55, BNO055_I2C_ADDR), // Initialize BNO055 with I2C address
    isInitialized(false),     // Set initialization flag to false
    lastUpdateTime(0) {       // Initialize last update time to 0
    memset(&data, 0, sizeof(IMUData));    // Clear IMUData structure
}

// Initializes the BNO055 sensor
// Returns true if initialization was successful, false otherwise.
bool IMUHandler::init() {
    if (!bno.begin()) {
        return false;    // Return false if the sensor fails to initialize
    }
    
    delay(1000); // Allow time for sensor to stabilize
    
    // Set the sensor to NDOF_FMC_OFF mode (Fusion Mode with Fast Magnetometer Calibration Off)
    bno.setMode(OPERATION_MODE_NDOF_FMC_OFF);
    // Enable external crystal for better accuracy
    bno.setExtCrystalUse(true);
    isInitialized = true;    // Mark the sensor as initialized
    return true;
}

// Updates the IMU data at a fixed interval (defined by UPDATE_INTERVAL)
// Returns true if data was successfully updated, false otherwise.
bool IMUHandler::update() {
    if (!isInitialized) return false;    // Return false if the sensor is not initialized
    
    unsigned long currentTime = millis();    // Get the current time
    if (currentTime - lastUpdateTime < UPDATE_INTERVAL) {
        return true; // Not time to update yet
    }
    
    // Get orientation data (yaw, pitch, roll)
    sensors_event_t event;
    bno.getEvent(&event);
    data.yaw = event.orientation.x;    // Yaw angle in degrees
    data.pitch = event.orientation.z;    // Pitch angle in degrees
    data.roll = event.orientation.y;    // Roll angle in degrees

    // Get gyroscope data (angular velocity)
    imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    // Get accelerometer data (linear acceleration)
    imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
    
    data.gyroX = gyro.x();    // Angular velocity around X-axis (degrees/second)
    data.gyroY = gyro.y();    // Angular velocity around Y-axis (degrees/second)
    data.gyroZ = gyro.z();    // Angular velocity around Z-axis (degrees/second)
    
    data.accelX = accel.x();    // Linear acceleration along X-axis (m/s^2)
    data.accelY = accel.y();    // Linear acceleration along Y-axis (m/s^2)
    data.accelZ = accel.z();    // Linear acceleration along Z-axis (m/s^2)

    // Get quaternion data
    imu::Quaternion quat = bno.getQuat();
    data.quatW = quat.w();  // Quaternion W component
    data.quatX = quat.x();  // Quaternion X component
    data.quatY = quat.y();  // Quaternion Y component
    data.quatZ = quat.z();  // Quaternion Z component

    // Normalize quaternion values to ensure they represent a valid rotation
    float norm = sqrt(data.quatW * data.quatW + 
                      data.quatX * data.quatX + 
                      data.quatY * data.quatY + 
                      data.quatZ * data.quatZ);
    
    data.quatW /= norm;
    data.quatX /= norm;
    data.quatY /= norm;
    data.quatZ /= norm;
   
    lastUpdateTime = currentTime;    // Update the last update time
    return true;
}

// Prints the calibration status of all components to the serial monitor.
// Also controls an RGB LED to indicate calibration status.
void IMUHandler::printCalibrationStatus() {
    if (!isInitialized) {
        Serial.println("IMU not initialized!");    // Print error message if not initialized
        return;
    }
    
    // Update calibration status for system, gyroscope, accelerometer, and magnetometer
    bno.getCalibration(&data.systemCal, &data.gyroCal, 
                       &data.accelCal, &data.magCal);
        
   // Blink red LED when system calibration is at its lowest level (0)
    if (data.systemCal == 0) {
        rgb.flashRedColor();    // Flash red LED using an external RGB handler
    } else {
        rgb.lightOff();    // Turn off RGB LED when calibration improves
    }
}

// Checks whether all components of the IMU are fully calibrated.
// Returns true if all components are calibrated (status == 3), false otherwise.
bool IMUHandler::isCalibrated() const {
    return (data.systemCal == 3 && data.gyroCal == 3 && 
            data.accelCal == 3 && data.magCal == 3);
}
