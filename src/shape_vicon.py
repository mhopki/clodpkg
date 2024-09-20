#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped

# Define the workspace boundaries
X_MIN, X_MAX = 0, 4
Y_MIN, Y_MAX = 0, 7

# Set up the environment
rospy.init_node('shapesvicon', anonymous=True)
way_pub = rospy.Publisher('/waypoints', PoseStamped, queue_size=100)

# Set initial value for 'once'
once = 10

# Continuously publish waypoints
rate = rospy.Rate(1000)  # Adjust the rate as needed (1 kHz)
while not rospy.is_shutdown():
    shape = []
    shape.append([-1,4,0.2])
    shape.append([1,4,0.2])
    shape.append([1,6,0.2])
    shape.append([-1,6,0.2])
    shape.append([-1,4,0.2])
    for i in range(1,len(shape) - 1):
        way_out = PoseStamped()
        way_out.pose.position.x = shape[i][0]
        way_out.pose.position.y = shape[i][1]
        way_out.pose.position.z = shape[i][2]
        way_pub.publish(way_out)
    rate.sleep()
