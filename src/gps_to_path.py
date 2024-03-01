#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import utm
from math import atan2

class GpsToPathConverter:
    def __init__(self):
        rospy.init_node('gps_to_path_converter', anonymous=True)

        self.path_pub = rospy.Publisher('/gps_path', Path, queue_size=10)
        self.gps_sub = rospy.Subscriber('/mavros/global_position/global', NavSatFix, self.gps_callback)

        self.path = Path()
        self.path.header.frame_id = 'map'  # Set the frame_id, adjust as needed
        self.first = None
        self.last = None

    def gps_callback(self, gps_msg):
        # Convert GPS coordinates to UTM coordinates
        easting, northing, _, _ = utm.from_latlon(gps_msg.latitude, gps_msg.longitude)
        if (self.first == None):
            self.first = [easting, northing, gps_msg.altitude]

        # Create PoseStamped message and set its position
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = rospy.Time.now()
        pose_stamped.pose.position.x = (easting - self.first[0])
        pose_stamped.pose.position.y = (northing - self.first[1])
        pose_stamped.pose.position.z = (gps_msg.altitude - self.first[2])

        # Convert GPS orientation to quaternion
        if self.last == None:
            quaternion = quaternion_from_euler(0, 0, 1)#gps_msg.track)
        else:
            # Calculate the yaw angle (rotation around the z-axis)
            delta_x = pose_stamped.pose.position.x - self.last.pose.position.x
            delta_y = pose_stamped.pose.position.y - self.last.pose.position.y
            yaw_angle = atan2(delta_y, delta_x)

            # Convert the yaw angle to a quaternion
            quaternion = quaternion_from_euler(0, 0, yaw_angle)

        pose_stamped.pose.orientation.x = quaternion[0]
        pose_stamped.pose.orientation.y = quaternion[1]
        pose_stamped.pose.orientation.z = quaternion[2]
        pose_stamped.pose.orientation.w = quaternion[3]

        # Add the PoseStamped to the Path
        self.path.header.stamp = rospy.Time.now()
        self.path.poses.append(pose_stamped)

        self.last = pose_stamped

        # Publish the Path
        self.path_pub.publish(self.path)

if __name__ == '__main__':
    try:
        converter = GpsToPathConverter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
