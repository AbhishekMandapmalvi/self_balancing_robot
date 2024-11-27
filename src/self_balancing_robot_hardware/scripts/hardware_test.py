#!/usr/bin/env python3

import roslibpy
import time
import sys
import os

# Add parent directory to path for imports
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

class HardwareTest:
    def __init__(self):
        # Initialize ROS client
        self.client = roslibpy.Ros(host='localhost', port=9090)
        self.client.run()

        # Create subscribers
        self.imu_subscriber = roslibpy.Topic(self.client, '/imu_data', 'sensor_msgs/Imu')
        self.left_encoder_subscriber = roslibpy.Topic(self.client, '/left_encoder', 'std_msgs/Int32')
        self.right_encoder_subscriber = roslibpy.Topic(self.client, '/right_encoder', 'std_msgs/Int32')
        
        # Create publisher
        self.motor_publisher = roslibpy.Topic(self.client, '/motor_cmd', 'std_msgs/Int32MultiArray')

        # Subscribe to topics
        self.imu_subscriber.subscribe(self.imu_callback)
        self.left_encoder_subscriber.subscribe(self.left_encoder_callback)
        self.right_encoder_subscriber.subscribe(self.right_encoder_callback)

    def imu_callback(self, message):
        orientation = message['orientation']
        angular_velocity = message['angular_velocity']
        linear_acceleration = message['linear_acceleration']
        
        print("\nIMU Data:")
        print(f"Orientation (w,x,y,z): {orientation['w']:.2f}, {orientation['x']:.2f}, "
              f"{orientation['y']:.2f}, {orientation['z']:.2f}")
        print(f"Angular Velocity (x,y,z): {angular_velocity['x']:.2f}, "
              f"{angular_velocity['y']:.2f}, {angular_velocity['z']:.2f}")
        print(f"Linear Acceleration (x,y,z): {linear_acceleration['x']:.2f}, "
              f"{linear_acceleration['y']:.2f}, {linear_acceleration['z']:.2f}")

    def left_encoder_callback(self, message):
        print(f"\nLeft Encoder Count: {message['data']}")

    def right_encoder_callback(self, message):
        print(f"Right Encoder Count: {message['data']}")

    def run_test(self):
        try:
            while self.client.is_connected:
                # Send test motor commands
                motor_msg = {'data': [100, -100]}
                self.motor_publisher.publish(roslibpy.Message(motor_msg))
                print("\nSent motor commands: Left=100, Right=-100")
                time.sleep(2)
                
                # Stop motors
                motor_msg = {'data': [0, 0]}
                self.motor_publisher.publish(roslibpy.Message(motor_msg))
                print("\nSent motor commands: Left=0, Right=0")
                time.sleep(2)

        except KeyboardInterrupt:
            self.cleanup()

    def cleanup(self):
        self.imu_subscriber.unsubscribe()
        self.left_encoder_subscriber.unsubscribe()
        self.right_encoder_subscriber.unsubscribe()
        self.motor_publisher.unadvertise()
        self.client.terminate()

if __name__ == '__main__':
    tester = HardwareTest()
    tester.run_test()