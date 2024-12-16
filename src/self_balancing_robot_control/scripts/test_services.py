#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32MultiArray
from std_srvs.srv import Trigger, TriggerResponse
import numpy as np

class SimpleController:
    def __init__(self):
        rospy.init_node('simple_controller', anonymous=True)
        
        # Create services
        self.pose_service = rospy.Service('get_pose', Trigger, self.handle_pose_request)
        self.cmd_service = rospy.Service('get_cmd_vel', Trigger, self.handle_cmd_request)
        
        # Subscriber
        rospy.Subscriber('sensor_data', Float32MultiArray, self.sensor_callback)
        
        # Robot state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0 #3.142
        
        # Preset commands
        self.latest_v_cmd = 0.0  # Constant linear velocity
        self.latest_w_cmd = 0.0  # Constant angular velocity
        
        self.rate = rospy.Rate(10)  # 10Hz update rate

    def handle_pose_request(self, req):
        response = TriggerResponse()
        try:
            pose_str = f"x: {self.x:.2f}, y: {self.y:.2f}, theta: {self.theta:.2f}"
            response.success = True
            response.message = pose_str
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response

    def handle_cmd_request(self, req):
        response = TriggerResponse()
        try:
            cmd_str = f"linear: {self.latest_v_cmd:.2f}, angular: {self.latest_w_cmd:.2f}"
            response.success = True
            response.message = cmd_str
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response

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
        getspeed_error = msg.data[14]
        getturn_error = msg.data[15]
        getTargetSpeed = msg.data[16]
        getTurnSpeed = msg.data[17]

        # Log data (for debugging)
        rospy.loginfo("getspeed_error: %.2f, getturn_error: %.2f, TargetSpeed: %.2f, TurnSpeed: %.2f, yaw: %.2f, gyroz: %.2f, left_vel: %.2f, right_vel: %.2f, left_dist: %.2f, right_dist: %.2f", 
                     getspeed_error, getturn_error, getTargetSpeed, getTurnSpeed, euler[2], gyro[2], left_vel, right_vel, left_dist, right_dist)

    def run(self):
        while not rospy.is_shutdown():
            self.rate.sleep()

if __name__ == '__main__':
    try:
        controller = SimpleController()
        controller.run()
    except rospy.ROSInterruptException:
        pass
