#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import tf.transformations
import cvxpy as cp
import numpy as np
import jax.numpy as jnp

class BalanceRobotController:
    def __init__(self):
        rospy.init_node('balance_robot_controller', anonymous=True)

        # Parameters for MPC
        self.Q = jnp.diag(jnp.array([1., 1., 0., 1.]))
        self.R = jnp.diag(jnp.array([1., 1.]))
        self.Qt = jnp.diag(jnp.array([1., 1., 0., 1.]))
        self.a_max = 0.5
        self.a_min = -0.5
        self.steer_max = 90.0*deg2rad
        self.steer_min = -90.0*deg2rad
        self.N_MPC_horizon = 10
        self.dt = ...

        # State variables
        self.current_state = None  # [x, y, theta (yaw), v]
        self.target_state = [...]  # Define your target trajectory

        # ROS Subscribers and Publishers
        rospy.Subscriber('odom', Odometry, self.odom_callback)
        self.cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

        self.rate = rospy.Rate(20)  # Control loop frequency

    def odom_callback(self, msg):
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        linear_velocity = msg.twist.twist.linear.x

        yaw = self.quaternion_to_euler(orientation)
        
        # Update current state: [x, y, yaw, v]
        self.current_state = [position.x, position.y, yaw, linear_velocity]

    def quaternion_to_euler(self, orientation):
        _, _, yaw = tf.transformations.euler_from_quaternion(
            [orientation.x, orientation.y, orientation.z, orientation.w]
        )
        return yaw

    def solve_mpc(self):
        problem = setup_trajopt(
            Q=self.Q,
            R=self.R,
            Qt=self.Qt,
            goal=self.target_state,
            angle_max=self.steer_max,
            angle_min=self.steer_min,
            acc_max=self.a_max,
            acc_min=self.a_min,
            MPC_horizon=self.N_MPC_horizon,
            dt=self.dt
        )
        
        update_initial_state(problem, self.current_state)
        
        problem.solve()
        
        return problem.variables()[1].value[:, 0]  # [steering_angle, acceleration]

    def publish_control(self, control_inputs):
        cmd_msg = Twist()
        
        cmd_msg.linear.x = self.current_state[3] + control_inputs[1] * self.dt  # Update velocity
        cmd_msg.angular.z = control_inputs[0]  # Steering angle as angular velocity
        
        self.cmd_pub.publish(cmd_msg)

    def setup_trajopt(Q, R, Qt, goal, angle_max, angle_min, acc_max, acc_min, MPC_horizon, dt):
        n = 4  # State dimension: [x, y, theta (yaw), v]
        m = 2  # Control dimension: [steering angle, acceleration]

        # Define optimization variables
        xs = cp.Variable((n, MPC_horizon + 1), name="states")
        u = cp.Variable((m, MPC_horizon), name="control")

        # Initial state as a parameter
        initial_state = cp.Parameter(n, name="initial_state")

        objective = 0.0
        constraints = []

        # Linearize dynamics around the goal state
        A, B, C = linearize_dynamics_autodifferentiation(continuous_time_unicycle_dynamics,
                                                        state0=goal,
                                                        control0=np.array([0.0, 0.0]))

        for t in range(MPC_horizon):
            objective += cp.quad_form((xs[:, t] - goal), Q) + cp.quad_form(u[:, t], R)
            constraints += [
                xs[:, t + 1] == xs[:, t] + A @ xs[:, t] + B @ u[:, t] + C,
                u[0, t] <= angle_max,
                u[0, t] >= angle_min,
                u[1, t] <= acc_max,
                u[1, t] >= acc_min
            ]

        # Terminal cost
        objective += cp.quad_form(xs[:, MPC_horizon] - goal, Qt)

        # Initial state constraint
        constraints += [xs[:, 0] == initial_state]

        problem = cp.Problem(cp.Minimize(objective), constraints)
        return problem

    def update_initial_state(problem, initial_state):
        """Update the initial state parameter before solving."""
        problem.param_dict["initial_state"].value = initial_state
        
    def run(self):
        while not rospy.is_shutdown():
            if self.current_state is not None:
                control_inputs = self.solve_mpc()
                self.publish_control(control_inputs)
            
            self.rate.sleep()

if __name__ == '__main__':
    try:
        controller = BalanceRobotController()
        controller.run()
    except rospy.ROSInterruptException:
        pass