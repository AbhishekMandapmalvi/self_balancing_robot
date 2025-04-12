#include <Arduino.h>
#include <ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include "pins_config.h"
#include "motor_controller.h"
#include "pid_controller.h"
#include "voltage.h"
#include "Ultrasonic.h"
#include "Rgb.h"

// Define ROS communication parameters
#define MAX_SUBSCRIBERS 2
#define MAX_PUBLISHERS 1
#define INPUT_SIZE 1024
#define OUTPUT_SIZE 1024

// Modify NodeHandle declaration to include custom buffer sizes and limits
ros::NodeHandle_<ArduinoHardware, MAX_SUBSCRIBERS, MAX_PUBLISHERS, INPUT_SIZE, OUTPUT_SIZE> nh;

// ROS message objects for publishing odometry and subscribing to velocity commands
nav_msgs::Odometry odom_msg;
geometry_msgs::Twist cmd_vel_msg;

// Global objects for controlling the balance car
MotorController motors;
EncoderHandler encoders;
IMUHandler imu_handler;
RGB rgb;
BalanceCar balance_car(motors, encoders, imu_handler);

// ROS publisher for odometry messages
ros::Publisher odom_pub("odom", &odom_msg);

// Callback function for processing velocity commands (cmd_vel)
// Updates the target speed and turn angle of the balance car based on received commands.
void cmdCallback(const geometry_msgs::Twist& cmd_msg) {
    float setting_car_speed = cmd_msg.linear.x;    // Extract linear speed from message
    float setting_turn_angle = cmd_msg.angular.z;    // Extract angular speed from message
    balance_car.setTargetSpeed(setting_car_speed);    // Update target speed in BalanceCar
    balance_car.setTurnAngle(setting_turn_angle);    // Update turn angle in BalanceCar
}

// ROS subscriber for velocity commands (cmd_vel)
ros::Subscriber<geometry_msgs::Twist> cmd_sub("cmd_vel", &cmdCallback);

// Timing variables for control loops
unsigned long previousBalanceTime = 0;    // Last execution time of the balance control loop
const unsigned long balanceInterval = 10;    // Interval for balance control loop (100Hz)
unsigned long previousPublishTime = 0;    // Last execution time of the communication loop
const unsigned long publishInterval = 50; // 20Hz for communication

void setup() {
    // Initialize ROS node and set baud rate for communication
    nh.getHardware()->setBaud(57600);
    nh.initNode();
    nh.advertise(odom_pub);    // Advertise the odometry topic
    nh.subscribe(cmd_sub);    // Subscribe to the velocity command topic
    
    // Initialize odometry message headers
    odom_msg.header.frame_id = "odom";    // Frame ID for odometry data
    odom_msg.child_frame_id = "base_link";    // Child frame ID representing the robot's base

    initializePins();    // Initialize pins configuration (defined in pins_config.h)
    rgb.initialize();    // Initialize RGB LED handler
    motors.init();       // Initialize motor controller
    encoders.init();     // Initialize encoder handler
    delay(1000);    // Delay to allow hardware initialization
     
    imu_handler.init();    // Initialize IMU sensor handler
    delay(2000);    // Delay to allow IMU stabilization
    voltageInit();    // Initialize voltage measurement system
}

void loop() {
    unsigned long currentTime = millis();    // Get current time in milliseconds
    
    // Balance control loop (100Hz)
    if (currentTime - previousBalanceTime >= balanceInterval) {
        previousBalanceTime = currentTime;
        encoders.update();    // Update encoder data (position and velocity)
        imu_handler.update(); // Update IMU data (orientation and motion)
        balance_car.update(); // Perform balance control calculations and update motor speeds
    }

    // Communication loop (20Hz)
    if (currentTime - previousPublishTime >= publishInterval) {
        previousPublishTime = currentTime;
        imu_handler.printCalibrationStatus();    // Print IMU calibration status to Serial Monitor
        rgb.blink(100);    // Blink RGB LED as a status indicator
        IMUData imu_data = imu_handler.getData();    // Retrieve IMU data
        
        // Set header timestamp for odometry message using ROS time synchronization
        odom_msg.header.stamp = nh.now();
        
        // Populate angular velocity data from IMU gyroscope readings
        odom_msg.twist.twist.angular.x = imu_data.gyroX;
        odom_msg.twist.twist.angular.y = imu_data.gyroY;
        odom_msg.twist.twist.angular.z = imu_data.gyroZ;

        // Populate orientation data using quaternion values from IMU readings
        odom_msg.pose.pose.orientation.x = imu_data.quatX;
        odom_msg.pose.pose.orientation.y = imu_data.quatY;
        odom_msg.pose.pose.orientation.z = imu_data.quatZ;
        odom_msg.pose.pose.orientation.w = imu_data.quatW;

        // Populate position data using encoder readings (assuming X and Y axes)
        odom_msg.pose.pose.position.x = encoders.getX();
        odom_msg.pose.pose.position.y = encoders.getY();
        //odom_msg.pose.pose.position.z = 0;

        // Populate linear velocity data using encoder readings (average of left and right wheels)
        odom_msg.twist.twist.linear.x = (encoders.getLeftVelocity() + encoders.getRightVelocity()) / 2.0;
        //odom_msg.twist.twist.linear.y = 0.0;
        //odom_msg.twist.twist.linear.z = 0.0;
        
        // Set covariance matrices for sensor fusion
        // Position covariance
        // Clear pose covariance matrix
        memset(odom_msg.pose.covariance, 0, sizeof(odom_msg.pose.covariance));

        // Clear twist covariance matrix
        memset(odom_msg.twist.covariance, 0, sizeof(odom_msg.twist.covariance));
        
        // Correct publish call
        odom_pub.publish(&odom_msg);
        voltageMeasure();    // Measure battery voltage and handle safety checks
        nh.spinOnce();    // Process incoming ROS messages and maintain communication with ROS master node
    }
}
