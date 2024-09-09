#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
import math

# Define the workspace boundaries
X_MIN, X_MAX = 0, 4
Y_MIN, Y_MAX = 0, 7

# Set up the environment
rospy.init_node('shapes', anonymous=True)
way_pub = rospy.Publisher('/waypoints', PoseStamped, queue_size=100)

# Continuously publish waypoints
rate = rospy.Rate(1000)  # Adjust the rate as needed (1 Hz)
while not rospy.is_shutdown():
    way_out = PoseStamped()
    way_out.pose.position.x = -1.0
    way_out.pose.position.y = 6.0
    way_out.pose.position.z = 0.1
    way_pub.publish(way_out)
    rate.sleep()
