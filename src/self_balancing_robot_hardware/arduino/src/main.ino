#include <ros.h>
#include <ros/time.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Int32.h>
#include "encoder_handler.h"
#include "imu_handler.h"
#include "motor_controller.h"

// ROS node handle
ros::NodeHandle nh;

// Create objects
EncoderHandler encoders;
IMUHandler imu;
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
    // Extract left and right motor speeds
    int leftSpeed = cmd_msg.data[0];
    int rightSpeed = cmd_msg.data[1];
    motors.setMotorSpeeds(leftSpeed, rightSpeed);
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
    if (!imu.init()) {
        // Handle IMU initialization failure
        while(1);
    }
    motors.init();
}

void loop() {
    static unsigned long last_publish_time = 0;
    unsigned long current_time = millis();
    
    // Update sensor readings
    encoders.update();
    imu.update();
    
    // Publish data every 10ms (100Hz)
    if (current_time - last_publish_time >= 10) {
        // Get IMU data
        IMUData imu_data = imu.getData();
        
        // Fill IMU message
        imu_msg.header.stamp = nh.now();
        imu_msg.header.frame_id = "imu_link";
        
        // Orientation (in quaternion)
        imu_msg.orientation.x = 0; // Convert from euler to quaternion if needed
        imu_msg.orientation.y = 0;
        imu_msg.orientation.z = 0;
        imu_msg.orientation.w = 1;
        
        // Angular velocity
        imu_msg.angular_velocity.x = imu_data.gyroX;
        imu_msg.angular_velocity.y = imu_data.gyroY;
        imu_msg.angular_velocity.z = imu_data.gyroZ;
        
        // Linear acceleration
        imu_msg.linear_acceleration.x = imu_data.linearAccelX;
        imu_msg.linear_acceleration.y = imu_data.linearAccelY;
        imu_msg.linear_acceleration.z = imu_data.linearAccelZ;
        
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