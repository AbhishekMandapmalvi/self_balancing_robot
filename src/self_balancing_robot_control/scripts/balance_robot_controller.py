#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist

class BalanceRobotController:
    def __init__(self):
        rospy.init_node('balance_robot_controller', anonymous=True)
        
        # Publisher for robot commands
        self.cmd_pub = rospy.Publisher('cmd_vel', Float32MultiArray, queue_size=10)
        
        # Subscriber for sensor data
        rospy.Subscriber('sensor_data', Float32MultiArray, self.sensor_callback)
        
        # Initialize messages
        self.cmd_msg = Float32MultiArray()
        self.cmd_msg.data = [0.0, 0.0]  # [linear_speed, angular_speed]
        
        # Subscribe to keyboard/joystick commands
        rospy.Subscriber('robot_cmd', Twist, self.cmd_callback)
        
        self.rate = rospy.Rate(100)  # 100Hz

    def sensor_callback(self, msg):
        # Process received sensor data
        accel = msg.data[0:3]    # accelerometer data
        gyro = msg.data[3:6]     # gyroscope data
        euler = msg.data[6:9]    # euler angles
        left_vel = msg.data[9]   # left wheel velocity
        right_vel = msg.data[10] # right wheel velocity
        left_dist = msg.data[11] # left wheel distance
        right_dist = msg.data[12] # right wheel distance
        us_distance = msg.data[13] - 7.00 # Distance

        # Log data (for debugging)
        rospy.loginfo("Euler angles (roll, pitch, yaw), Encoder(left_vel, right_vel, left_dist, right_dist, us_distance): %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f", 
                     euler[0], euler[1], euler[2], left_vel, right_vel, left_dist, right_dist, us_distance)

    def cmd_callback(self, twist_msg):
        # Convert Twist message to Float32MultiArray
        self.cmd_msg.data[0] = twist_msg.linear.x   # linear speed
        self.cmd_msg.data[1] = twist_msg.angular.z  # angular speed
        
        # Publish command to Arduino
        self.cmd_pub.publish(self.cmd_msg)

    def run(self):
        while not rospy.is_shutdown():
            self.rate.sleep()

if __name__ == '__main__':
    try:
        controller = BalanceRobotController()
        controller.run()
    except rospy.ROSInterruptException:
        pass