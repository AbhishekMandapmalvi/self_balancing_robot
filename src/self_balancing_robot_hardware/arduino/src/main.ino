#include <ros.h>
#include <ros/time.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int32MultiArray.h>
#include <Adafruit_BNO055.h>

#include <Arduino.h>
#include <stdint.h>
#include <Wire.h>

#include "encoder_handler.h"
#include "imu_handler.h"
#include "motor_controller.h"
#include "pins_config.h"

// ROS node handle
ros::NodeHandle nh;

// Create objects
EncoderHandler encoders;
IMUHandler imu_sensor;
MotorController motors;

// Messages
sensor_msgs::Imu imu_msg;
std_msgs::Int32 left_encoder_msg;
std_msgs::Int32 right_encoder_msg;

// Publishers
ros::Publisher imu_pub("imu_data", &imu_msg);
ros::Publisher left_encoder_pub("left_encoder", &left_encoder_msg);
ros::Publisher right_encoder_pub("right_encoder", &right_encoder_msg);

// Callback function for motor commands
void motorCommandCallback(const std_msgs::Int32MultiArray& cmd_msg) {
    if(cmd_msg.data_length >= 2) {
    // Extract left and right motor speeds
    int leftSpeed = cmd_msg.data[0];
    int rightSpeed = cmd_msg.data[1];
    motors.setMotorSpeeds(leftSpeed, rightSpeed);
    }
}

// Subscriber
ros::Subscriber<std_msgs::Int32MultiArray> motor_sub("motor_cmd", &motorCommandCallback);

void setup() {
    // Initialize serial communication
    Serial.begin(57600);  // Standard baud rate for rosserial

    // Initialize ROS node
    nh.initNode();

    // Initialize pins
    initializePins();
    
    // Advertise publishers
    nh.advertise(imu_pub);
    nh.advertise(left_encoder_pub);
    nh.advertise(right_encoder_pub);
    
    // Subscribe to motor commands
    nh.subscribe(motor_sub);
    
    // Initialize hardware
    encoders.init();
    if (!imu_sensor.init()) {
        // Handle IMU initialization failure
        Serial.println("BNO055 initialization failed!");
        while(1);
    }
    motors.init();
    delay(1000); // Allow IMU to stabilize
}

void loop() {
    static unsigned long last_publish_time = 0;
    unsigned long current_time = millis();
    
    // Update sensor readings
    encoders.update();
    imu_sensor.update();
    
    // Publish data every 10ms (100Hz)
    if (current_time - last_publish_time >= 10) {
        // Get IMU data
        IMUData imu_data = imu_sensor.getData();
        
        // Fill IMU message
        imu_msg.header.stamp = nh.now();
        imu_msg.header.frame_id = "imu_link";
        
        // Convert Euler angles to quaternion
        float roll = imu_data.roll * M_PI / 180.0;
        float pitch = imu_data.pitch * M_PI / 180.0;
        float yaw = imu_data.yaw * M_PI / 180.0;

        // Calculate quaternion
        float cy = cos(yaw * 0.5);
        float sy = sin(yaw * 0.5);
        float cp = cos(pitch * 0.5);
        float sp = sin(pitch * 0.5);
        float cr = cos(roll * 0.5);
        float sr = sin(roll * 0.5);

        imu_msg.orientation.w = cr * cp * cy + sr * sp * sy;
        imu_msg.orientation.x = sr * cp * cy - cr * sp * sy;
        imu_msg.orientation.y = cr * sp * cy + sr * cp * sy;
        imu_msg.orientation.z = cr * cp * sy - sr * sp * cy;

        // Angular velocity
        imu_msg.angular_velocity.x = imu_data.gyroX;
        imu_msg.angular_velocity.y = imu_data.gyroY;
        imu_msg.angular_velocity.z = imu_data.gyroZ;
        
        // Linear acceleration
        imu_msg.linear_acceleration.x = imu_data.accelX;
        imu_msg.linear_acceleration.y = imu_data.accelY;
        imu_msg.linear_acceleration.z = imu_data.accelZ;
        
        // Encoder data
        left_encoder_msg.data = encoders.getLeftCount();
        right_encoder_msg.data = encoders.getRightCount();
        
        // Publish messages
        imu_pub.publish(&imu_msg);
        left_encoder_pub.publish(&left_encoder_msg);
        right_encoder_pub.publish(&right_encoder_msg);
        
        last_publish_time = current_time;
    }
    
    // Process ROS messages
    nh.spinOnce();
}