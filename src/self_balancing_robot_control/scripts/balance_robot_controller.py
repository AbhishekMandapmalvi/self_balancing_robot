#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

class BalanceRobotController:
    def __init__(self):
        rospy.init_node('balance_robot_controller_main', anonymous=True)
        
        # Subscriber for odometry data
        rospy.Subscriber('odom', Odometry, self.odom_callback)

        # Publisher for cmd_vel (to send commands to Arduino)
        self.cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

        self.rate = rospy.Rate(20)  # 20Hz

    def odom_callback(self, msg):
        # Process received odometry data
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        linear_velocity = msg.twist.twist.linear
        angular_velocity = msg.twist.twist.angular

        # Log data (for debugging)
        rospy.loginfo(
            "Position: (x: %.2f, y: %.2f), Orientation: (x: %.2f, y: %.2f, z: %.2f, w: %.2f), "
            "Linear Velocity: (x: %.2f), Angular Velocity: (z: %.2f)",
            position.x, position.y,
            orientation.x, orientation.y, orientation.z, orientation.w,
            linear_velocity.x,
            angular_velocity.z
        )

    def send_default_cmd(self):
        # Create a Twist message with default values (zeros)
        cmd_msg = Twist()
        cmd_msg.linear.x = 0.0   # Default speed
        cmd_msg.angular.z = 0.0  # Default turn angle

        # Publish the message to cmd_vel topic
        self.cmd_pub.publish(cmd_msg)

    def run(self):
        while not rospy.is_shutdown():
            # Send default command message at regular intervals
            self.send_default_cmd()
            self.rate.sleep()

if __name__ == '__main__':
    try:
        controller = BalanceRobotController()
        controller.run()
    except rospy.ROSInterruptException:
        pass
