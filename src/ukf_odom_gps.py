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
from tf.transformations import quaternion_slerp, quaternion_multiply, quaternion_inverse
import numpy as np
from filterpy.kalman import MerweScaledSigmaPoints, UnscentedKalmanFilter
from tf.transformations import quaternion_from_euler
import tf2_ros
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import TransformStamped

class UkfLocalizationNode:
    def __init__(self):
        rospy.init_node('ukf_localization_example')

        # Subscribe to Odometry and NavSatFix messages
        rospy.Subscriber('/camera/odom/sample', Odometry, self.odom_callback)
        rospy.Subscriber('/mavros/global_position/global', NavSatFix, self.gps_callback)

        self.broadcaster = tf2_ros.TransformBroadcaster()

        # Initial transform (you may set this based on your current configuration)
        self.transform = TransformStamped()
        self.transform.header.frame_id = 'map'
        self.transform.child_frame_id = 'camera_frame'

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

        self.translated = False
        self.reori = 20
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

            delta_x2 = self.last_gps_msg.pose.position.x - pose_stamped.pose.position.x 
            delta_y2 = self.last_gps_msg.pose.position.y - pose_stamped.pose.position.y
            delta_z2 = self.last_gps_msg.pose.position.z - pose_stamped.pose.position.z

            # Calculate the yaw angle (rotation around the z-axis)
            yaw_angle = atan2(delta_y, delta_x)

            # Convert the yaw angle to a quaternion
            quaternion = quaternion_from_euler(0, 0, yaw_angle)

            if abs(delta_x) > 0 or abs(delta_x2) > 0 or abs(delta_y) > 0 or abs(delta_y2) > 0:
                self.translated = True


            # If it's the first pose message, just return a default quaternion
            #if delta_x == 0 and delta_y == 0:
            #    quaternion = quaternion_from_euler(0, 0, 1)

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
        # UKF Fusion logic (example: simple averaging)
        fused_localization = Odometry()
        fused_localization.header.stamp = rospy.Time.now()
        fused_localization.header.frame_id = 'map'
        #print("ran")

        if self.last_odom_msg is not None and self.last_gps_msg is not None:
            odom_position = self.last_odom_msg.pose.pose.position
            gps_position = self.last_gps_msg.pose.position

            #print(self.last_gps_msg.pose.orientation)
            if (self.reori and self.translated):
                gps_orientation = self.last_gps_msg.pose.orientation
                camera_orientation = self.last_odom_msg.pose.pose.orientation

                #new_q = gps_orientation + (gps_orientation - camera_orientation) * (self.reori / 20)

                gps_orientation = np.array([gps_orientation.x, gps_orientation.y, gps_orientation.z, gps_orientation.w])
                camera_orientation = np.array([camera_orientation.x, camera_orientation.y, camera_orientation.z, camera_orientation.w])

                # Calculate the orientation difference
                orientation_difference = quaternion_multiply(gps_orientation, quaternion_inverse(camera_orientation))
                print(orientation_difference)

                #new_q = np.array([new_q.x, new_q.y, new_q.z, new_q.w])

                """
                # Calculate the differences in each component
                diff_x = gps_orientation[0] - camera_orientation[0]
                diff_y = gps_orientation[1] - camera_orientation[1]
                diff_z = gps_orientation[2] - camera_orientation[2]
                diff_w = gps_orientation[3] - camera_orientation[3]

                # Scale the differences by the interpolation factor
                scaled_diff_x = diff_x * (self.reori / 20)
                scaled_diff_y = diff_y * (self.reori / 20)
                scaled_diff_z = diff_z * (self.reori / 20)
                scaled_diff_w = diff_w * (self.reori / 20)

                # Apply the scaled differences to gps_orientation
                gps_orientation[0] += scaled_diff_x
                gps_orientation[1] += scaled_diff_y
                gps_orientation[2] += scaled_diff_z
                gps_orientation[3] += scaled_diff_w

                # Update the transform quaternion
                

                # Update the transform quaternion
                self.transform = TransformStamped()
                self.transform.header.frame_id = 'map'
                self.transform.child_frame_id = 'base_link'
                #self.transform.transform.rotation.x = rot1.x# * (self.reori / 20)
                #self.transform.transform.rotation.y = rot1.y# * (self.reori / 20)
                #self.transform.transform.rotation.z = rot1.z# * (self.reori / 20)
                #self.transform.transform.rotation.w = rot1.w# * (self.reori / 20)
                #rotin = np.array([rot1.x, rot1.y, rot1.z, rot1.w])
                #self.transform.transform.rotation = Quaternion(*rot1)
                self.transform.transform.rotation = geometry_msgs.msg.Quaternion(*gps_orientation)"""
                
                """
                self.transform = TransformStamped()
                self.transform.header.frame_id = 'map'
                self.transform.child_frame_id = 'base_link'
                self.transform.transform.rotation.x = new_q[0]
                self.transform.transform.rotation.y = new_q[1]
                self.transform.transform.rotation.z = new_q[2]
                self.transform.transform.rotation.w = new_q[3]"""

                #"""
                self.transform = TransformStamped()
                self.transform.header.frame_id = 'map'
                self.transform.child_frame_id = 'base_link'
                self.transform.transform.rotation.x = orientation_difference[0]
                self.transform.transform.rotation.y = orientation_difference[1]
                self.transform.transform.rotation.z = orientation_difference[2]
                self.transform.transform.rotation.w = orientation_difference[3]#"""

                #transformed_quaternion = do_transform_quaternion(quaternion, transform_stamped)

                self.broadcaster.sendTransform(self.transform)

                self.reori -= 1
                self.translated = False

            # Use Unscented Kalman Filter for state estimation
            points = MerweScaledSigmaPoints(4, alpha=0.1, beta=2.0, kappa=-1.0)
            ukf = UnscentedKalmanFilter(dim_x=4, dim_z=3, dt=1.0, fx=self.fx, hx=self.hx, points=points)

            # Initialize state and covariance
            ukf.x = np.array([odom_position.x, odom_position.y, gps_position.x, gps_position.y])
            ukf.P *= 1e-2

            # Process and measurement noise
            ukf.R = np.diag([0.001, 0.001, 0.001])  # Tune this based on your sensor characteristics
            ukf.Q = np.diag([8.0, 8.0, 8.0, 8.0])  # Tune this based on your system dynamics

            # Perform filter prediction and update
            ukf.predict()
            ukf.update([gps_position.x, gps_position.y, gps_position.z])

            # Create an Odometry message for the fused localization
            fused_localization = Odometry()
            fused_localization.header.stamp = rospy.Time.now()
            #fused_localization.header.stamp = self.last_odom_msg.header.stamp
            fused_localization.header.frame_id = 'map'

            # Update the pose from the UKF state
            fused_localization.pose.pose.position.x = ukf.x[0]
            fused_localization.pose.pose.position.y = ukf.x[1]
            fused_localization.pose.pose.position.z = ukf.x[2]

            # Update the orientation from the last Odometry message
            fused_localization.pose.pose.orientation = self.last_odom_msg.pose.pose.orientation

            # Set covariance (modify as needed)
            fused_localization.pose.covariance = [0.1] * 36  # Example covariance

        return fused_localization

    def fx(self, x, dt):
        # State transition function
        # This function predicts the next state based on the current state
        return np.array([x[0] + x[2] * dt, x[1] + x[3] * dt, x[2], x[3]])

    def hx(self, x):
        # Measurement function
        # This function maps the state space to the measurement space
        return np.array([x[0], x[1], x[2]])


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
    ukf_node = UkfLocalizationNode()
    #print("WTF")
    ukf_node.spin()
