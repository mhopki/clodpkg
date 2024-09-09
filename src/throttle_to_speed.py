#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Joy                # For publishing joystick messages
import inputs                                 # To handle input events (like joystick)
from nav_msgs.msg import Odometry             # For receiving odometry data
from geometry_msgs.msg import PoseStamped     # For handling pose data
import math                                   # Math operations
import random                                 # For generating random numbers
from std_msgs.msg import Float32MultiArray    # For multi-array messages
import numpy as np                            # For numerical operations
import argparse

# Callback function for the first topic
def callback_topic1(msg):
    #rospy.loginfo(f"Received from topic1: {msg.data}")
    # You can also combine or process messages here as needed
    print("THROTTLE: ", msg.axes[7])

# Callback function for the second topic
def callback_topic2(msg):
    #rospy.loginfo(f"Received from topic2: {msg.data}")
    # You can also combine or process messages here as needed
    print("SPEED: ", msg.twist.twist.linear.y)

def listener():
    # Initialize the ROS node
    rospy.init_node('subscribe_two_topics', anonymous=True)

    # Subscribe to the first topic
    rospy.Subscriber('/joy', Joy, callback_topic1)

    # Subscribe to the second topic
    rospy.Subscriber('/vicon/BEAST_02/odom', Odometry, callback_topic2)

    # Keep the node running and listening to the topics
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
