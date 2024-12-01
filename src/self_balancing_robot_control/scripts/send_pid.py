#!/usr/bin/env python3

import rospy
from self_balancing_robot_msgs.msg import PIDParams

def pid_publisher():
    # Initialize the ROS node
    rospy.init_node('pid_parameter_publisher', anonymous=True)
    
    # Create a publisher for PID parameters
    pub = rospy.Publisher('pid_params', PIDParams, queue_size=10)
    
    # Set the publishing rate (Hz)
    rate = rospy.Rate(1)  # 1 Hz

    # Create a message instance
    pid_msg = PIDParams()

    while not rospy.is_shutdown():
        # Update message values
        pid_msg.kp = 10.0
        pid_msg.ki = 0.1
        pid_msg.kd = 0.05
        pid_msg.kp_pos = 8.0
        pid_msg.ki_pos = 0.0
        pid_msg.kd_pos = 0.2

        # Publish the message
        pub.publish(pid_msg)
        rospy.loginfo("Published PID parameters")
        
        # Sleep to maintain the rate
        rate.sleep()

if __name__ == '__main__':
    try:
        pid_publisher()
    except rospy.ROSInterruptException:
        pass