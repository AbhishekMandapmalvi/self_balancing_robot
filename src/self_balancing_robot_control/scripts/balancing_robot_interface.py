#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import Int32, Int32MultiArray, MultiArrayDimension
import numpy as np

class BalancingRobotInterface:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('balancing_robot_interface')
        
        # Publishers for motor commands
        self.motor_pub = rospy.Publisher('motor_cmd', Int32MultiArray, queue_size=10)
        
        # Subscribers for sensor data
        rospy.Subscriber('imu_data', Imu, self.imu_callback)
        rospy.Subscriber('left_encoder', Int32, self.left_encoder_callback)
        rospy.Subscriber('right_encoder', Int32, self.right_encoder_callback)
        
        # Store sensor data
        self.imu_data = None
        self.left_encoder_count = 0
        self.right_encoder_count = 0

    def imu_callback(self, msg):
        self.imu_data = msg
        # Extract orientation (quaternion)
        w = msg.orientation.w
        x = msg.orientation.x
        y = msg.orientation.y
        z = msg.orientation.z
        
        # Extract angular velocity
        gyro_x = msg.angular_velocity.x
        gyro_y = msg.angular_velocity.y
        gyro_z = msg.angular_velocity.z
        
        # Extract linear acceleration
        accel_x = msg.linear_acceleration.x
        accel_y = msg.linear_acceleration.y
        accel_z = msg.linear_acceleration.z

    def left_encoder_callback(self, msg):
        self.left_encoder_count = msg.data

    def right_encoder_callback(self, msg):
        self.right_encoder_count = msg.data

    def send_motor_command(self, left_speed, right_speed):
        # Create Int32MultiArray message
        msg = Int32MultiArray()
        msg.layout.dim = [MultiArrayDimension()]
        msg.layout.dim[0].label = "motor_speeds"
        msg.layout.dim[0].size = 2
        msg.layout.dim[0].stride = 2
        msg.data = [left_speed, right_speed]
        
        # Publish the message
        self.motor_pub.publish(msg)

    def run(self):
        rate = rospy.Rate(100)  # 100Hz control loop
        
        while not rospy.is_shutdown():
            if self.imu_data is not None:
                # Example: Send test motor commands
                # Replace this with your control algorithm
                left_speed = 0  # Speed range: -255 to 255
                right_speed = 0  # Speed range: -255 to 255
                self.send_motor_command(left_speed, right_speed)
            
            rate.sleep()

if __name__ == '__main__':
    try:
        interface = BalancingRobotInterface()
        interface.run()
    except rospy.ROSInterruptException:
        pass