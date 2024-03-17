#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped

class OdomToPathConverter:
    def __init__(self):
        rospy.init_node('odom_to_path_converter')

        self.path_pub = rospy.Publisher('/odom_path', Path, queue_size=100)
        self.path2_pub = rospy.Publisher('/fused_path', Path, queue_size=100)
        self.odom_sub = rospy.Subscriber('/camera/odom/sample', Odometry, self.odom_callback)
        self.odom2_sub = rospy.Subscriber('/fused_localization', Odometry, self.odom2_callback)

        self.path_msg = Path()
        self.path_msg.header.frame_id = 'camera_odom_frame'  # Adjust the frame_id accordingly

        self.path2_msg = Path()
        self.path2_msg.header.frame_id = 'base_link'  # Adjust the frame_id accordingly

    def odom_callback(self, odom_msg):
        pose = PoseStamped()
        pose.header = odom_msg.header
        pose.pose = odom_msg.pose.pose

        self.path_msg.poses.append(pose)
        self.path_pub.publish(self.path_msg)

    def odom2_callback(self, odom_msg):
        pose = PoseStamped()
        pose.header = odom_msg.header
        pose.pose = odom_msg.pose.pose

        self.path2_msg.poses.append(pose)
        self.path2_pub.publish(self.path2_msg)

if __name__ == '__main__':
    try:
        odom_to_path_converter = OdomToPathConverter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
