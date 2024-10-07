#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped

def main():
    rospy.init_node('waypoint_sender', anonymous=True)
    way_pub = rospy.Publisher('/waypoints', PoseStamped, queue_size=100)

    # Prompt user for x and y values
    try:
        x = float(input("Enter the x coordinate for the waypoint: "))
        y = float(input("Enter the y coordinate for the waypoint: "))
    except ValueError:
        rospy.logerr("Invalid input! Please enter numeric values for x and y.")
        return

    # Create the PoseStamped message with the provided x and y coordinates
    way_out = PoseStamped()
    way_out.pose.position.x = x
    way_out.pose.position.y = y

    # Publish the waypoint
    rospy.loginfo(f"Publishing waypoint: x={x}, y={y}")
    way_pub.publish(way_out)

    #rospy.spin()  # Keeps the node alive to continuously process incoming data (if needed)

if __name__ == '__main__':
    main()
