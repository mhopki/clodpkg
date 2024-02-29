#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

import matplotlib.pyplot as plt
import random
import math
import numpy as np
#from scipy.interpolate import CubicSpline

#from scipy.optimize import minimize

# Define the workspace boundaries
X_MIN, X_MAX = 0, 4
Y_MIN, Y_MAX = 0, 7


# Set up the environment
rospy.init_node('shapess', anonymous=True)
global way_pub
way_pub = rospy.Publisher('/waypoints', PoseStamped, queue_size=100)

true_path = []
shape_t = 0
#0 == square
if shape_t == 0:
    true_path.append([0,0])
    true_path.append([3,0])
    true_path.append([3,3])
    true_path.append([0,3])
    true_path.append([0,0])

o_off_x = 0
o_off_y = 0

#true_path
path_good = True
if (path_good == True):
    way_path = []
    way_out = PoseStamped()
    way_out.pose.position.x = true_path[0][0] + o_off_x
    way_out.pose.position.y = true_path[0][1] + o_off_y
    print("publish")
    #print(way_out)
    way_pub.publish(way_out)
    #print(way_pub.publish(way_out))
    #way_path.append(Node(true_path[0,0], true_path[0,1]))

    last_x = true_path[0][0] + o_off_x
    last_y = true_path[0][1] + o_off_y
    for i in range(1,len(true_path) - 1):
        if (math.sqrt(((true_path[i][0] + o_off_x) - last_x)**2 + ((true_path[i][1] + o_off_y) - last_y)**2) >= 0.1):
            way_out = PoseStamped()
            way_out.pose.position.x = true_path[i][0] + o_off_x
            way_out.pose.position.y = true_path[i][1] + o_off_y
            print("publish")
            #print(way_out)
            way_pub.publish(way_out)
            #print(way_pub.publish(way_out))
            #way_path.append(Node(true_path[i,0], true_path[i,1]))
            #plt.plot([last_x, smoothed_path[i].x - 2], [last_y, smoothed_path[i].y + 2], 'y-')
            last_x = true_path[i][0] + o_off_x
            last_y = true_path[i][1] + o_off_y

    way_out = PoseStamped()
    way_out.pose.position.x = true_path[-1][0] + o_off_x
    way_out.pose.position.y = true_path[-1][1] + o_off_y
    print("publish")
    #print(way_out)
    way_pub.publish(way_out)
    #print(way_pub.publish(way_out))
    #way_path.append(Node(true_path[-1][0], true_path[-1][1]))
    #plt.plot([last_x, smoothed_path[i].x - 2], [last_y, smoothed_path[i].y + 2], 'y-')
    last_x = true_path[i][0] + o_off_x
    last_y = true_path[i][1] + o_off_y

#print("start: ", start.x, start.y)
#for node in path:
#    print("node: ", node.x, node.y)
#print("goal: ", goal.x, goal.y)