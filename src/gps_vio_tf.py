#!/usr/bin/env python3

import rospy
import tf

if __name__ == '__main__':
    rospy.init_node('frame_alignment_node')

    listener = tf.TransformListener()

    rate = rospy.Rate(1.0)

    while not rospy.is_shutdown():
        try:
            # Wait for the transformation to be available
            listener.waitForTransform("map", "base_link", rospy.Time(), rospy.Duration(4.0))

            # Get the transformation
            (translation, rotation) = listener.lookupTransform("map", "base_link", rospy.Time(0))

            # Now you have the translation and rotation
            print("Translation:", translation)
            print("Rotation:", rotation)

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logerr("Unable to get transformation.")

        rate.sleep()
