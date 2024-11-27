#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import Int32, Int32MultiArray
import time

class HardwareTest:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('hardware_tester')

        # Create subscribers
        rospy.Subscriber('/imu_data', Imu, self.imu_callback)
        rospy.Subscriber('/left_encoder', Int32, self.left_encoder_callback)
        rospy.Subscriber('/right_encoder', Int32, self.right_encoder_callback)
        
        # Create publisher
        self.motor_publisher = rospy.Publisher('/motor_cmd', Int32MultiArray, queue_size=10)

    def imu_callback(self, message):
        print("\nIMU Data:")
        print(f"Orientation (w,x,y,z): {message.orientation.w:.2f}, {message.orientation.x:.2f}, "
              f"{message.orientation.y:.2f}, {message.orientation.z:.2f}")
        print(f"Angular Velocity (x,y,z): {message.angular_velocity.x:.2f}, "
              f"{message.angular_velocity.y:.2f}, {message.angular_velocity.z:.2f}")
        print(f"Linear Acceleration (x,y,z): {message.linear_acceleration.x:.2f}, "
              f"{message.linear_acceleration.y:.2f}, {message.linear_acceleration.z:.2f}")

    def left_encoder_callback(self, message):
        print(f"\nLeft Encoder Count: {message.data}")

    def right_encoder_callback(self, message):
        print(f"Right Encoder Count: {message.data}")

    def run_test(self):
        rate = rospy.Rate(0.5)  # 0.5 Hz for 2-second intervals
        try:
            while not rospy.is_shutdown():
                # Send test motor commands
                motor_msg = Int32MultiArray(data=[100, -100])
                self.motor_publisher.publish(motor_msg)
                print("\nSent motor commands: Left=100, Right=-100")
                rate.sleep()
                
                # Stop motors
                motor_msg = Int32MultiArray(data=[0, 0])
                self.motor_publisher.publish(motor_msg)
                print("\nSent motor commands: Left=0, Right=0")
                rate.sleep()

        except KeyboardInterrupt:
            self.cleanup()

    def cleanup(self):
        # Stop motors before shutting down
        motor_msg = Int32MultiArray(data=[0, 0])
        self.motor_publisher.publish(motor_msg)
        rospy.signal_shutdown("User requested shutdown")

if __name__ == '__main__':
    tester = HardwareTest()
    tester.run_test()