#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int16

class MotorController:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('motor_controller', anonymous=True)
        
        # Publishers for motor commands
        self.left_motor_pub = rospy.Publisher('left_motor_cmd', Int16, queue_size=10)
        self.right_motor_pub = rospy.Publisher('right_motor_cmd', Int16, queue_size=10)
        
        # Encoder values
        self.left_encoder_value = 0
        self.right_encoder_value = 0
        
        # Subscribe to encoder topics
        rospy.Subscriber('left_encoder', Int16, self.left_encoder_callback)
        rospy.Subscriber('right_encoder', Int16, self.right_encoder_callback)
        
        # PWM value for motors (adjust as needed)
        self.motor_pwm = 100  # Set a moderate PWM value
        
    def left_encoder_callback(self, msg):
        self.left_encoder_value = msg.data
        
    def right_encoder_callback(self, msg):
        self.right_encoder_value = msg.data
        
    def run(self):
        rate = rospy.Rate(50)  # 10Hz
        target_value = 1000
                
        while not rospy.is_shutdown():
            left_msg = Int16()
            right_msg = Int16()
            
            # Left motor control
            if abs(self.left_encoder_value) < target_value:
                left_msg.data = self.motor_pwm
            else:
                left_msg.data = 0
                rospy.loginfo("Left Motor Target Reached!")
            
            # Right motor control
            if abs(self.right_encoder_value) < target_value:
                right_msg.data = self.motor_pwm
            else:
                right_msg.data = 0
                rospy.loginfo("Right Motor Target Reached!")
            rospy.loginfo(f"Left Motor Running - Encoder: {self.left_encoder_value}, Right Motor Running - Encoder: {self.right_encoder_value}")
            # Publish PWM values
            self.left_motor_pub.publish(left_msg)
            self.right_motor_pub.publish(right_msg)
            
            # Stop program only if both motors have reached target
            if abs(self.left_encoder_value) >= target_value and abs(self.right_encoder_value) >= target_value:
                rospy.loginfo("Both motor have reached their targets!")
                break
            
            rate.sleep()

if __name__ == '__main__':
    try:
        controller = MotorController()
        controller.run()
    except rospy.ROSInterruptException:
        pass