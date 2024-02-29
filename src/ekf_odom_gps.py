#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from robot_localization.srv import SetPose
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import euler_from_quaternion
from diagnostic_msgs.msg import DiagnosticArray
import utm
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from math import atan2
from tf.transformations import quaternion_slerp, quaternion_multiply
import numpy as np

class EkfLocalizationNode:
    def __init__(self):
        rospy.init_node('ekf_localization_example')

        # Subscribe to Odometry and NavSatFix messages
        rospy.Subscriber('/camera/odom/sample', Odometry, self.odom_callback)
        rospy.Subscriber('/mavros/global_position/global', NavSatFix, self.gps_callback)

        # Advertise the fused localization estimate
        self.localization_pub = rospy.Publisher('/fused_localization', Odometry, queue_size=10)

        #print("ello nigga")

        # Set up a service to manually set the initial pose of the filter (optional)
        #rospy.wait_for_service('/set_pose')
        #self.set_pose_service = rospy.ServiceProxy('/set_pose', SetPose)

        # Internal state variables
        self.fgpm = None
        self.last_odom_msg = None
        self.last_gps_msg = None
        self.first_gps_msg = None  # Initialize first_gps_msg here
        #print("init has run")

    def odom_callback(self, odom_msg):
        # Store the last received Odometry message
        self.last_odom_msg = odom_msg

    def gps_callback(self, gps_msg):
        # Store the last received NavSatFix message
        easting, northing, _, _ = utm.from_latlon(gps_msg.latitude, gps_msg.longitude)
        altitude = gps_msg.altitude

        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = rospy.Time.now()
        pose_stamped.pose.position.x = easting
        pose_stamped.pose.position.y = northing
        pose_stamped.pose.position.z = gps_msg.altitude

        # Convert GPS orientation to quaternion
        quaternion = quaternion_from_euler(0, 0, 1)#gps_msg.track)

        pose_stamped.pose.orientation.x = quaternion[0]
        pose_stamped.pose.orientation.y = quaternion[1]
        pose_stamped.pose.orientation.z = quaternion[2]
        pose_stamped.pose.orientation.w = quaternion[3]

        if self.first_gps_msg is None:
            self.first_gps_msg = PoseStamped()
            self.first_gps_msg.header.stamp = rospy.Time.now()
            self.first_gps_msg.pose.position.x = easting
            self.first_gps_msg.pose.position.y = northing
            self.first_gps_msg.pose.position.z = gps_msg.altitude
            # Convert GPS orientation to quaternion
            quaternion = quaternion_from_euler(0, 0, 1)#gps_msg.track)

            self.first_gps_msg.pose.orientation.x = quaternion[0]
            self.first_gps_msg.pose.orientation.y = quaternion[1]
            self.first_gps_msg.pose.orientation.z = quaternion[2]
            self.first_gps_msg.pose.orientation.w = quaternion[3]

        #set gps_orientation
        if self.last_gps_msg != None and self.first_gps_msg != None:
            # Calculate the orientation quaternion from the last and current positions
            delta_x = (easting - self.first_gps_msg.pose.position.x) - self.last_gps_msg.pose.position.x
            delta_y = (northing - self.first_gps_msg.pose.position.y) - self.last_gps_msg.pose.position.y
            delta_z = (gps_msg.altitude - self.first_gps_msg.pose.position.z) - self.last_gps_msg.pose.position.z

            # Calculate the yaw angle (rotation around the z-axis)
            yaw_angle = atan2(delta_y, delta_x)

            # Convert the yaw angle to a quaternion
            quaternion = quaternion_from_euler(0, 0, yaw_angle)


            # If it's the first pose message, just return a default quaternion
            if delta_x == 0 and delta_y == 0:
                quaternion = quaternion_from_euler(0, 0, 1)

        #self.last_gps_msg = pose_stamped - self.first_gps_msg
        self.last_gps_msg = PoseStamped()
        self.last_gps_msg.header.stamp = rospy.Time.now()
        self.last_gps_msg.pose.position.x = (easting - self.first_gps_msg.pose.position.x)
        self.last_gps_msg.pose.position.y = (northing - self.first_gps_msg.pose.position.y)
        self.last_gps_msg.pose.position.z = (gps_msg.altitude - self.first_gps_msg.pose.position.z)
        # Convert GPS orientation to quaternion
        #quaternion = quaternion_from_euler(0, 0, 1)#gps_msg.track)

        self.last_gps_msg.pose.orientation.x = quaternion[0]
        self.last_gps_msg.pose.orientation.y = quaternion[1]
        self.last_gps_msg.pose.orientation.z = quaternion[2]
        self.last_gps_msg.pose.orientation.w = quaternion[3]
        #print("ok")


    def fuse_odom_and_gps(self):
        # EKF Fusion logic (example: simple averaging)
        fused_localization = Odometry()
        fused_localization.header.stamp = rospy.Time.now()
        fused_localization.header.frame_id = 'map'
        #print("ran")

        if self.last_odom_msg is not None and self.last_gps_msg is not None:
            fused_localization.header.stamp = self.last_odom_msg.header.stamp
            #fused_localization.header.frame_id = self.last_gps_msg.header.frame_id
            # Example: Use Odometry pose and average it with GPS position
            odom_position = self.last_odom_msg.pose.pose.position
            gps_position = self.last_gps_msg.pose.position

            # Simple averaging
            self.last_gps_msg.header.stamp = rospy.Time.now()
            ratio_o = 0.8
            fused_localization.pose.pose.position.x = (1-ratio_o) * (odom_position.x) + ratio_o * (gps_position.x)
            fused_localization.pose.pose.position.y = (1-ratio_o) * (odom_position.y) + ratio_o * (gps_position.y)
            fused_localization.pose.pose.position.z = (1-ratio_o) * (odom_position.z) + ratio_o  * (gps_position.z)

            # Set orientation from Odometry (modify as needed)
            q1m = self.last_odom_msg.pose.pose.orientation
            q2m = self.last_gps_msg.pose.orientation
            quaternion1 = np.array([q1m.x, q1m.y, q1m.z, q1m.w])
            quaternion2 = np.array([q2m.x, q2m.y, q2m.z, q2m.w])
            #print(quaternion1)
            #print(quaternion2)
            quaternion1 /= np.linalg.norm(quaternion1)
            quaternion2 /= np.linalg.norm(quaternion2)
            ratio = 0.2

            # SLERP interpolation
            interpolated_quaternion = quaternion_slerp(quaternion1, quaternion2, ratio)

            #return interpolated_quaternion
            #fused_localization.pose.pose.orientation = interpolated_quaternion#self.last_gps_msg.pose.orientation#self.last_odom_msg.pose.pose.orientation
            fused_localization.pose.pose.orientation.x = interpolated_quaternion[0]
            fused_localization.pose.pose.orientation.y = interpolated_quaternion[1]
            fused_localization.pose.pose.orientation.z = interpolated_quaternion[2]
            fused_localization.pose.pose.orientation.w = interpolated_quaternion[3]

            # Set covariance (modify as needed)
            fused_localization.pose.covariance = [0.1] * 36  # Example covariance

        return fused_localization

    def spin(self):
        rate = rospy.Rate(30)
        # Manually set the initial pose (optional)
        # This can be useful if your robot starts in a known position
        initial_pose = Odometry()
        initial_pose.pose.pose.position.x = 0.0
        initial_pose.pose.pose.position.y = 0.0
        initial_pose.pose.pose.position.z = 0.0
        initial_pose.pose.pose.orientation.w = 1.0
        #self.set_pose_service(initial_pose)
        #print("init")


        # Run the node
        #rate = rospy.Rate(30)  # Adjust the rate according to your sensor data rates
        while not rospy.is_shutdown():
            # Perform EKF fusion and publish the result
            fused_localization = self.fuse_odom_and_gps()
            self.localization_pub.publish(fused_localization)
            #print("fuse")

            rate.sleep()
            #print("repeat")

if __name__ == '__main__':
    #print("AYOO")
    ekf_node = EkfLocalizationNode()
    #print("WTF")
    ekf_node.spin()
