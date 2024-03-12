#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from cv_bridge import CvBridge
import sensor_msgs.point_cloud2 as pc2
import numpy as np
import cv2
import struct
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Bool
import pyoctree
import rospy
from octomap_msgs.msg import Octomap
from octomap_msgs.srv import BoundingBoxQuery
from octomap_msgs.srv import GetOctomap
from sensor_msgs.msg import Range
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

class WaypointListener:
    def __init__(self):
        rospy.init_node('waypoint_listener', anonymous=True)
        self.bridge = CvBridge()

        # Subscriber for depth image and camera info
        rospy.Subscriber('/waypoints', PoseStamped, self.wayp_callback)

        # Create a Rate object to control the loop frequency
        self.rate = rospy.Rate(1000)  # 10 Hz

        # Main loop
        self.run()

    def run(self):
        while not rospy.is_shutdown():
            self.rate.sleep()

    def wayp_callback(self, data):

        print("way_coords: ", data.pose.position.x, data.pose.position.y)
        #self.waypoints.append(data)

if __name__ == '__main__':
    try:
        converter = WaypointListener()
        #rospy.spin()
    except rospy.ROSInterruptException:
        pass
