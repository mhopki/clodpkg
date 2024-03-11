#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Float32MultiArray

class ImageSegmentation:
    def __init__(self):
        rospy.init_node('image_segmentation_node', anonymous=True)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/camera/image_raw', Image, self.image_callback)
        self.image_pub2 = rospy.Publisher('/segmented_image/hsv', Image, queue_size=10)
        self.image_pub = rospy.Publisher('/segmented_image/rgb', Image, queue_size=10)

        # New publisher for world coordinates
        self.world_coordinates_pub = rospy.Publisher('/object_world_coordinates', Float32MultiArray, queue_size=10)

        # Focal length of the camera (you need to replace this with the actual focal length)
        self.focal_length = 2300.0  # Example value, replace with the actual focal length

    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        except CvBridgeError as e:
            rospy.logerr(e)
            return

        segmented_image, center_coordinates, seg_hsv, whv = self.segment_image(cv_image)

        try:
            # Publish the HSV version of the image
            self.image_pub2.publish(self.bridge.cv2_to_imgmsg(seg_hsv, 'bgr8'))
        except CvBridgeError as e:
            rospy.logerr(e)

        # Draw a blue bounding box around the segmented area
        cv2.rectangle(cv_image, (center_coordinates[0] - 50, center_coordinates[1] - 50),
                      (center_coordinates[0] + 50, center_coordinates[1] + 50), (255, 0, 0), 2)

        # Display center coordinates or "No center" in the top-left corner
        text = f"Center: {center_coordinates}" if center_coordinates != (0, 0) else "No center"
        cv2.putText(cv_image, text, (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.5, (255, 255, 255), 4, cv2.LINE_AA)

        # Calculate and display distance estimate based on the observed height
        observed_height = (center_coordinates[1] + whv[1]) - (center_coordinates[1] - whv[1])  # Height of the bounding box
        actual_height = 11  # Actual height of the object in centimeters
        if whv != (0, 0): 
            distance_estimate = (actual_height * self.focal_length) / observed_height
        else:
            distance_estimate = 0
        distance_text = f"Distance: {distance_estimate:.2f} cm" if whv != (0, 0) else "No object"
        cv2.putText(cv_image, distance_text, (10, 80),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.5, (255, 255, 255), 4, cv2.LINE_AA)

        # Calculate world coordinates based on distance and image coordinates
        if whv != (0, 0):
            y_world = (center_coordinates[0] - cv_image.shape[1] / 2) * (distance_estimate / self.focal_length)
            z_world = (center_coordinates[1] - cv_image.shape[0] / 2) * (distance_estimate / self.focal_length)
            x_world = distance_estimate
            world_coordinates_text = f"World Coordinates: X={x_world:.2f} cm, Y={y_world:.2f} cm, Z={z_world:.2f} cm"
            cv2.putText(cv_image, world_coordinates_text, (10, 130),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.5, (255, 255, 255), 4, cv2.LINE_AA)

            # Publish world coordinates to the topic
            world_coordinates_msg = Float32MultiArray(data=[x_world, y_world, z_world])
            self.world_coordinates_pub.publish(world_coordinates_msg)

        # Mirror the image vertically and horizontally
        cv_image = cv2.flip(cv_image, 0)
        cv_image = cv2.flip(cv_image, 1)

        try:
            # Publish the RGB version of the image
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, 'bgr8'))
        except CvBridgeError as e:
            rospy.logerr(e)

    def segment_image(self, image):
        # Convert image to HSV
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Define blue color range in HSV
        lower_blue = np.array([0, 150, 50])  # Original lower range
        upper_blue = np.array([10, 255, 255])  # Original upper range

        # Create a mask for the blue color
        blue_mask = cv2.inRange(hsv_image, lower_blue, upper_blue)

        # Create a black image
        hsv_image_white = np.zeros_like(hsv_image)

        # Set blue pixels to white in the segmented image
        hsv_image_white[blue_mask != 0] = [255, 255, 255]

        # Find contours in the mask
        contours, _ = cv2.findContours(blue_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            # Get the largest contour (assuming it's the segmented area)
            largest_contour = max(contours, key=cv2.contourArea)

            # Get the bounding box of the largest contour
            x, y, w, h = cv2.boundingRect(largest_contour)

            # Check if the object is at least 20x20 pixels
            if w >= 70 and h >= 70:
                # Get the center coordinates of the bounding box
                center_x = x + w // 2
                center_y = y + h // 2

                # Draw the segmented area on the image
                cv2.drawContours(image, [largest_contour], -1, (0, 255, 0), 2)

                return hsv_image, (center_x, center_y), hsv_image_white, (w, h)

        return hsv_image, (0, 0), hsv_image_white, (0, 0)

if __name__ == '__main__':
    try:
        segmentation_node = ImageSegmentation()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
