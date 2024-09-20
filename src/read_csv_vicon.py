#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
import numpy as np 
from numpy import genfromtxt
import matplotlib.pyplot as plt 
import sys


class CSVPlanner:
    def __init__(self):
        # Set up the environment
        x = 0

    def read_from_csv(self, file_name) : 
        my_data = genfromtxt(file_name, delimiter=',')
        my_data = my_data[1:]
        times = my_data[:,0]
        positions = my_data[:,1:3]
        velocities = my_data[:,3:5]
        return times, positions, velocities

def send_waypoints(path, way_pub):
    for i in range(2):
        max_i = len(path) - 1
        way_out = PoseStamped()
        way_out.pose.position.x = path[max_i - i][0]
        way_out.pose.position.y = path[max_i - i][1]
        way_pub.publish(way_out)
        print(path[max_i - i])

def main():
    rospy.init_node('csv_planner', anonymous=True)
    way_pub = rospy.Publisher('/waypoints', PoseStamped, queue_size=100)
    # Continuously publish waypoints
    #rate = rospy.Rate(1000)  # Adjust the rate as needed (1 kHz)

    planner = CSVPlanner()
    times, positions, vels = planner.read_from_csv('traj_0.csv')
    print(times[0], positions[0], vels[0])

    if (True):
    #while not rospy.is_shutdown():
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
            print(way_pub.publish(PoseStamped()))
        #rate.sleep()

    send_waypoints(positions, way_pub)

if __name__ == "__main__" :
    main()
