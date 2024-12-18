#include <Arduino.h>
#include <EEPROM.h>
#include <ros.h>
#include <std_msgs/Float32MultiArray.h>
#include "pins_config.h"
#include "motor_controller.h"
#include "pid_controller.h"
#include "voltage.h"
#include "Ultrasonic.h"
#include "Rgb.h"

ros::NodeHandle nh;

std_msgs::Float32MultiArray sensor_msg;
float sensor_data[14];

// Fix 1: Correct publisher declaration
ros::Publisher sensor_pub("sensor_data", &sensor_msg);

// Global objects
MotorController motors;
EncoderHandler encoders;
IMUHandler imu_handler;
RGB rgb;
BalanceCar balance_car(motors, encoders, imu_handler);

void cmdCallback(const std_msgs::Float32MultiArray& cmd_msg) {
    float setting_car_speed = cmd_msg.data[0];
    float setting_turn_angle = cmd_msg.data[1];
    balance_car.setTargetSpeed(setting_car_speed);
    balance_car.setTurnAngle(setting_turn_angle);
}

ros::Subscriber<std_msgs::Float32MultiArray> cmd_sub("cmd_vel", &cmdCallback);

unsigned long previousBalanceTime = 0;
const unsigned long balanceInterval = 10;
unsigned long previousPublishTime = 0;
const unsigned long publishInterval = 50; // 20Hz for communication

void setup() {
    nh.initNode();
    // Fix 2: Correct advertise call
    nh.advertise(sensor_pub);
    nh.subscribe(cmd_sub);
    Serial.begin(115200);

    sensor_msg.data = sensor_data;
    sensor_msg.data_length = 14;

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

        sensor_data[0] = imu_data.accelX;
        sensor_data[1] = imu_data.accelY;
        sensor_data[2] = imu_data.accelZ;
        sensor_data[3] = imu_data.gyroX;
        sensor_data[4] = imu_data.gyroY;
        sensor_data[5] = imu_data.gyroZ;
        sensor_data[6] = imu_data.roll;
        sensor_data[7] = imu_data.pitch;
        sensor_data[8] = imu_data.yaw;
        sensor_data[9] = encoders.getLeftVelocity();
        sensor_data[10] = encoders.getRightVelocity();
        sensor_data[11] = encoders.getLeftDistance();
        sensor_data[12] = encoders.getRightDistance();
        sensor_data[13] = getDistance();

        // Correct publish call
        sensor_pub.publish(&sensor_msg);
        voltageMeasure();
        nh.spinOnce();
    }
}