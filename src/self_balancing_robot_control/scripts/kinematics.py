#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32MultiArray, Float32
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger, TriggerResponse
import math
import numpy as np
from scipy.optimize import minimize
    
class MPCController:
    def __init__(self):
        rospy.init_node('kinematics_controller', anonymous=True)
        
        # Publishers
        self.cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.pose_pub = rospy.Publisher('robot_pose', Float32MultiArray, queue_size=10)
        
        # Create services instead of publishers
        #self.pose_service = rospy.Service('get_pose', Trigger, self.handle_pose_request)
        #self.cmd_service = rospy.Service('get_cmd_vel', Trigger, self.handle_cmd_request)
        
        # Subscribers
        rospy.Subscriber('sensor_data', Float32MultiArray, self.sensor_callback)
        
        # Robot state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.prev_time = rospy.Time.now()
        self.prev_left_dist = 0.0
        self.prev_right_dist = 0.0
        
        # Latest computed commands
        self.latest_v_cmd = 0.0
        self.latest_w_cmd = 0.0

        # obstacle avoidance parameters
        self.obstacle_weight = 2.0  # Weight for obstacle avoidance
        self.safe_distance = 20.0    # Minimum safe distance in centimeters
        self.us_distance = float('inf')  # Initialize with large value

        # Target state (can be updated via another subscriber if needed)
        self.x_target = 1.0  # Example target
        self.y_target = 0.0
        self.theta_target = np.deg2rad(90.0)
        
        # MPC parameters
        self.N = 40  # Longer horizon to account for PID dynamics
        self.dt = 0.05  # 20 Hz control interval
        self.v_max = 0.50  # Conservative velocity limit
        self.w_max = 2*np.pi  # Conservative angular velocity limit
        
        # Cost function weights
        self.Q = np.diag([1.0, 1.0, 0.8])  # state error weights
        self.R = np.diag([0.2, 0.2])  # control input weights
        
        # PID response simulation parameters
        self.pid_tau = 0.3  # PID response time constant
        
        self.rate = rospy.Rate(10)  # 10Hz for MPC updates

    def handle_pose_request(self, req):
        response = TriggerResponse()
        try:
            pose_str = f"x: {self.x_target}, y: {self.y_target}, theta: {self.theta_target}"
            response.success = True
            response.message = pose_str
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response

    def handle_cmd_request(self, req):
        response = TriggerResponse()
        try:
            cmd_str = f"linear: {self.latest_v_cmd}, angular: {self.latest_w_cmd}"
            #rospy.loginfo(f"Sending command: {cmd_str}")
            response.success = True
            response.message = cmd_str
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response

    def simulate_pid_response(self, command, current, dt):
        # Simple first-order response simulation
        tau = self.pid_tau
        response = current + (command - current) * (1 - np.exp(-dt/tau))
        return response

    def predict_state(self, current_state, v_cmd, w_cmd, current_v, current_w, dt):
        # Simulate PID response
        v_actual = self.simulate_pid_response(v_cmd, current_v, dt)
        w_actual = self.simulate_pid_response(w_cmd, current_w, dt)
        
        x, y, theta = current_state
        next_state = np.array([
            x + v_actual * np.cos(theta) * dt,
            y + v_actual * np.sin(theta) * dt,
            theta + w_actual * dt
        ])
        return next_state, v_actual, w_actual

    def optimize_trajectory(self, current_state, current_v, current_w):
        target_state = np.array([self.x_target, self.y_target, self.theta_target])
        
        def objective(u):
            cost = 0
            state = current_state
            v_current = current_v
            w_current = current_w
            
            for i in range(self.N):
                v_cmd = u[i*2]
                w_cmd = u[i*2 + 1]
                
                next_state, v_next, w_next = self.predict_state(state, v_cmd, w_cmd, v_current, w_current, self.dt)
                
                state_error = next_state - target_state
                control_input = np.array([v_cmd, w_cmd])
                
                cost += state_error.T @ self.Q @ state_error
                cost += control_input.T @ self.R @ control_input
                
                # Obstacle avoidance cost
                if self.us_distance < self.safe_distance:
                    obstacle_cost = self.obstacle_weight * (
                        (self.safe_distance - self.us_distance)**2 * 
                        abs(v_cmd * np.cos(state[2]))  # Only penalize forward motion
                    )
                    cost += obstacle_cost

                state = next_state
                v_current = v_next
                w_current = w_next
            
            return cost

        # Initialize optimization
        u0 = np.zeros(2*self.N)
        bounds = [(-self.v_max, self.v_max), (-self.w_max, self.w_max)]*self.N
        
        result = minimize(objective, u0, bounds=bounds, method='SLSQP')
        return result.x[0], result.x[1]

    def sensor_callback(self, msg):
        # Validate message data length
        #if len(msg.data) < 18:  # We expect at least 16 elements
            # rospy.logerr(f"Received sensor data with insufficient length: {len(msg.data)}")
        #    return
        #try:
            # Process sensor data
        current_time = rospy.Time.now()
        dt = (current_time - self.prev_time).to_sec()
        self.prev_time = current_time

        # Extract sensor data
        euler_yaw = np.deg2rad(msg.data[8])    # euler yaw angle
        left_vel = msg.data[9]     # left wheel velocity
        right_vel = msg.data[10]   # right wheel velocity
        w_imu = (msg.data[6]) * (np.pi/180)
        self.us_distance = msg.data[13] - 7.00 # distance over ultrasonic sensor
        
        # Calculate current velocities
        current_v = (left_vel + right_vel) / 2
        current_w = w_imu  # Using imu yaw rate as current angular velocity
        
        # Current state
        current_state = np.array([self.x, self.y, self.theta])
        
        # Get optimal control inputs
        v_cmd, w_cmd = self.optimize_trajectory(current_state, current_v, current_w)
        
        # Modify commands based on obstacle proximity
        if self.us_distance < self.safe_distance:
            v_cmd = min(v_cmd, 0.2)  # Limit forward velocity when close to obstacle
        
        # Store latest commands instead of publishing
        self.latest_v_cmd = v_cmd
        self.latest_w_cmd = w_cmd

        # Publish command
        #cmd_msg = Twist()
        #cmd_msg.linear.x = v_cmd
        #cmd_msg.angular.z = w_cmd
        #self.cmd_pub.publish(cmd_msg)
        
        # More accurate state update using encoder distances
        left_dist = msg.data[11]  # left wheel distance
        right_dist = msg.data[12]  # right wheel distance
        self.theta = euler_yaw  # Direct angle from IMU

        # Calculate distance changes
        delta_left = left_dist - self.prev_left_dist
        delta_right = right_dist - self.prev_right_dist

        # Calculate displacement
        wheel_distance = (delta_left + delta_right) / 2
        # Update position
        self.x += wheel_distance * np.cos(self.theta)
        self.y += wheel_distance * np.sin(self.theta)
        
        # Update previous distances
        self.prev_left_dist = left_dist
        self.prev_right_dist = right_dist

        # Publish pose
        #pose_msg = Float32MultiArray()
        #pose_msg.data = [self.x, self.y, self.theta]
        #self.pose_pub.publish(pose_msg)
        
        distance_to_target = np.sqrt((self.x_target - self.x)**2 + (self.y_target - self.y)**2)
        angle_error = np.rad2deg(self.theta_target - self.theta)
            
            # rospy.loginfo("Current Position (x,y,theta): %.2f, %.2f, %.2f°", 
            #             self.x, self.y, np.rad2deg(self.theta))
            # rospy.loginfo("Target Position (x,y,theta): %.2f, %.2f, %.2f°", 
            #             self.x_target, self.y_target, np.rad2deg(self.theta_target))
            # rospy.loginfo("Distance to target: %.2f m, Angle error: %.2f°", 
            #             distance_to_target, angle_error)
            # rospy.loginfo("Commands (v,w): %.2f, %.2f", v_cmd, w_cmd)
            # rospy.loginfo("Obstacle distance: %.2f cm", self.us_distance)
            # rospy.loginfo("Delta distances: left=%.4f, right=%.4f", delta_left, delta_right)
            # rospy.loginfo("Wheel distance: %.4f", wheel_distance)
            # rospy.loginfo("State update: dx=%.4f, dy=%.4f", 
            #                 wheel_distance * np.cos(self.theta),
            #                 wheel_distance * np.sin(self.theta))
            # rospy.loginfo("Raw sensor data: %s", str(msg.data))
        
        # except IndexError as e:
            # rospy.logerr(f"Index error in sensor callback: {e}")
        # except Exception as e:
            # rospy.logerr(f"Error in sensor callback: {e}")
    
    def run(self):
        while not rospy.is_shutdown():
            self.rate.sleep()

if __name__ == '__main__':
    try:
        controller = MPCController()
        controller.run()
    except rospy.ROSInterruptException:
        pass
