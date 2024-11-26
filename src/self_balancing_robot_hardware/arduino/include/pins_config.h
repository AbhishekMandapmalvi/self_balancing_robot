#ifndef PINS_CONFIG_H
#define PINS_CONFIG_H

// Motor Driver Pins
static const int AIN1 = 7;          // Left motor direction
static const int PWMA_LEFT = 5;     // Left motor PWM
static const int BIN1 = 12;         // Right motor direction
static const int PWMB_RIGHT = 6;    // Right motor PWM
static const int STBY_PIN = 8;      // Standby pin

// Motor Driver Pins
static const int MOTOR_LEFT_DIR = AIN1;      // Pin 7
static const int MOTOR_LEFT_PWM = PWMA_LEFT; // Pin 5
static const int MOTOR_RIGHT_DIR = BIN1;     // Pin 12
static const int MOTOR_RIGHT_PWM = PWMB_RIGHT; // Pin 6
static const int MOTOR_STANDBY = STBY_PIN;   // Pin 8

// Encoder Pins
static const int ENCODER_LEFT_A_PIN = 2;
static const int ENCODER_LEFT_B_PIN = 3;
static const int ENCODER_RIGHT_A_PIN = 4;
static const int ENCODER_RIGHT_B_PIN = 9;

// Encoder Pins
static const int ENCODER_LEFT_A = ENCODER_LEFT_A_PIN;   // Pin 2
static const int ENCODER_LEFT_B = ENCODER_LEFT_B_PIN;   // Pin 3
static const int ENCODER_RIGHT_A = ENCODER_RIGHT_A_PIN; // Pin 4
static const int ENCODER_RIGHT_B = ENCODER_RIGHT_B_PIN; // Pin 9

// I2C Pins for BNO055 (Arduino Uno/Nano)
static const int I2C_SDA = A4;  // Analog pin 4
static const int I2C_SCL = A5;  // Analog pin 5

// Constants
static const uint8_t BNO055_I2C_ADDR = 0x28;  // Default I2C address for BNO055

void initializePins() {
    // Motor pins
    pinMode(MOTOR_LEFT_DIR, OUTPUT);
    pinMode(MOTOR_LEFT_PWM, OUTPUT);
    pinMode(MOTOR_RIGHT_DIR, OUTPUT);
    pinMode(MOTOR_RIGHT_PWM, OUTPUT);
    pinMode(MOTOR_STANDBY, OUTPUT);
    
    // Encoder pins
    pinMode(ENCODER_LEFT_A, INPUT_PULLUP);
    pinMode(ENCODER_LEFT_B, INPUT_PULLUP);
    pinMode(ENCODER_RIGHT_A, INPUT_PULLUP);
    pinMode(ENCODER_RIGHT_B, INPUT_PULLUP);
    
    // Enable motor driver
    digitalWrite(MOTOR_STANDBY, HIGH);
    
    // I2C pins will be automatically configured by Wire library
    Wire.begin();
}

#endif