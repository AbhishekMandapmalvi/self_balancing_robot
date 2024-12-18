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

//ros::NodeHandle nh;
#define MAX_SUBSCRIBERS 2
#define MAX_PUBLISHERS 1
#define INPUT_SIZE 1024
#define OUTPUT_SIZE 1024

// Modify NodeHandle declaration
ros::NodeHandle_<ArduinoHardware, MAX_SUBSCRIBERS, MAX_PUBLISHERS, INPUT_SIZE, OUTPUT_SIZE> nh;

nav_msgs::Odometry odom_msg;
geometry_msgs::Twist cmd_vel_msg;

// Global objects
MotorController motors;
EncoderHandler encoders;
IMUHandler imu_handler;
RGB rgb;
BalanceCar balance_car(motors, encoders, imu_handler);

// Correct publisher declaration
ros::Publisher odom_pub("odom", &odom_msg);

void cmdCallback(const geometry_msgs::Twist& cmd_msg) {
    float setting_car_speed = cmd_msg.linear.x;
    float setting_turn_angle = cmd_msg.angular.z;
    balance_car.setTargetSpeed(setting_car_speed);
    balance_car.setTurnAngle(setting_turn_angle);
}

ros::Subscriber<geometry_msgs::Twist> cmd_sub("cmd_vel", &cmdCallback);

unsigned long previousBalanceTime = 0;
const unsigned long balanceInterval = 10;
unsigned long previousPublishTime = 0;
const unsigned long publishInterval = 50; // 20Hz for communication

void setup() {
    nh.getHardware()->setBaud(57600);
    nh.initNode();
    nh.advertise(odom_pub);    
    nh.subscribe(cmd_sub);
    
    // Initialize message headers
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "base_link";

    initializePins();
    rgb.initialize();
    motors.init();
    encoders.init();
    delay(1000);
     
    imu_handler.init();
    delay(2000);    
    voltageInit();
}

void loop() {
    unsigned long currentTime = millis();
    
    // Balance control loop (100Hz)
    if (currentTime - previousBalanceTime >= balanceInterval) {
        previousBalanceTime = currentTime;
        encoders.update();
        imu_handler.update();
        balance_car.update();
    }

    // Communication loop (20Hz)
    if (currentTime - previousPublishTime >= publishInterval) {
        previousPublishTime = currentTime;
        imu_handler.printCalibrationStatus();
        rgb.blink(100);
        IMUData imu_data = imu_handler.getData();

        // Set header timestamp
        odom_msg.header.stamp = nh.now();
        
        // Angular velocities (from IMU)
        odom_msg.twist.twist.angular.x = imu_data.gyroX;
        odom_msg.twist.twist.angular.y = imu_data.gyroY;
        odom_msg.twist.twist.angular.z = imu_data.gyroZ;

        odom_msg.pose.pose.orientation.x = imu_data.quatX;
        odom_msg.pose.pose.orientation.y = imu_data.quatY;
        odom_msg.pose.pose.orientation.z = imu_data.quatZ;
        odom_msg.pose.pose.orientation.w = imu_data.quatW;

        odom_msg.pose.pose.position.x = encoders.getX();
        odom_msg.pose.pose.position.y = encoders.getY();
        //odom_msg.pose.pose.position.z = 0;

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
        voltageMeasure();
        nh.spinOnce();
    }
}
