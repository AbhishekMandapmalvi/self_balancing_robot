#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32MultiArray

class BalanceRobotController:
    def __init__(self):
        rospy.init_node('balance_robot_controller_main', anonymous=True)
                
        # Subscriber for sensor data
        rospy.Subscriber('sensor_data', Float32MultiArray, self.sensor_callback)

        self.rate = rospy.Rate(20)  # 100Hz

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
        Speed_error = msg.data[14]
        TurnSpeed_error = msg.data[15]
        TargetSpeed = msg.data[16]
        TurnSpeed = msg.data[17]

        # Log data (for debugging)
        rospy.loginfo("w: %.2f, Yaw: %.2f, TargetSpeed: %.2f, TurnSpeed: %.2f, Encoder(left_vel: %.2f, right_vel: %.2f, left_dist: %.2f, right_dist: %.2f, Speed_error: %.2f, TurnSpeed_error: %.2f)", 
                     gyro[2], euler[2], TargetSpeed, TurnSpeed, left_vel, right_vel, left_dist, right_dist, Speed_error, TurnSpeed_error)

    def run(self):
        while not rospy.is_shutdown():
            self.rate.sleep()

if __name__ == '__main__':
    try:
        controller = BalanceRobotController()
        controller.run()
    except rospy.ROSInterruptException:
        pass