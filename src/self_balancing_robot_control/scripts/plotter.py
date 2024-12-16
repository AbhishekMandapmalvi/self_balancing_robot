#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32MultiArray, Float32
from geometry_msgs.msg import Twist
import matplotlib.pyplot as plt
from collections import deque
import time

#Initialize global variables
pitch_buffer = []
leftv_buffer = []
rightv_buffer = []
start_time = None
collection_time = 10

def callback(msg):
    global start_time
    if start_time is None:
        start_time = time.time()

    # store data with timestamps
    current_time = time.time()
    if  current_time - start_time < collection_time:    
        pitch_buffer.append((current_time, msg.data[8]))
        leftv_buffer.append((current_time, msg.data[9]))
        rightv_buffer.append((current_time, msg.data[10]))

    else:
        plot_data()
        # Stop collecting after 10 seconds
        rospy.signal_shutdown("Data collection complete")

def plot_data():
    pitch_times, pitch_values = zip(*pitch_buffer)
    leftv_times, leftv_values = zip(*leftv_buffer)
    rightv_times, rightv_values = zip(*rightv_buffer)
    
    fig, ax = plt.subplots(3,1, figsize=(10,8))

    ax[0].plot(pitch_times, pitch_values)
    ax[0].set_title('Pitch')
    ax[0].set_xlabel('Time (s)')
    ax[0].set_ylabel('Pitch vs TIme')

    ax[1].plot(leftv_times, leftv_values)
    ax[1].set_title('Left Wheel Velocity')
    ax[1].set_xlabel('Time (s)')
    ax[1].set_ylabel('Left Wheel Velocity vs TIme')

    ax[2].plot(rightv_times, rightv_values)
    ax[2].set_title('Right Wheel Velocity')
    ax[2].set_xlabel('Time (s)')
    ax[2].set_ylabel('Right Wheel Velocity vs TIme')
    
    plt.tight_layout()
    plt.show()

def listener():
    rospy.init_node('data_listener', anonymous=True)
    # Subscriber
    rospy.Subscriber('sensor_data', Float32MultiArray, callback)
    
    # Keep the node alive until manually stopped or shutdown is signaled
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
        plot_data()
    except rospy.ROSInterruptException:
        pass