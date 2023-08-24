#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Joy
import inputs
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import math


# Initialize the ROS node and publisher


# Create a Joy message

button_codes = {
	0: "BTN_EAST",
	1: "BTN_SOUTH",
	2: "BTN_WEST",
	3: "BTN_NORTH",
	4: "BTN_TR",
	5: "BTN_TL",
	6: "BTN_THUMBR",
	7: "BTN_THUMBL",
	8: "BTN_START",
	9: "BTN_SELECT"
}

axis_codes = {
	0: "ABS_X",
	1: "ABS_Y",
	2: "ABS_RX",
	3: "ABS_RY",
	4: "ABS_HAT0X",
	5: "ABS_HAT0Y",
	6: "ABS_Z",
	7: "ABS_RZ",
}

r_btc = {value: key for key, value in button_codes.items()}
r_atc = {value: key for key, value in axis_codes.items()}



class POCont:
	def __init__(self):
		rospy.init_node('po_controller', anonymous=True)
		self.joy_pub = rospy.Publisher('/joy', Joy, queue_size=100)
		self.odom_sub = rospy.Subscriber('/vicon/BEAST/odom', Odometry, self.odom_callback, queue_size=1, tcp_nodelay=True)
		#pose_sub = rospy.Subscriber('/vicon/BEAST/pose', PoseStamped, pose_callback, queue_size=1, tcp_nodelay=True)
		self.last_received_time = rospy.Time.now()

		self.joy_msg = Joy()
		self.joy_msg.axes = [0.0] * 8
		self.joy_msg.buttons = [0] * 12
		self.last_joy_message = Joy()

		self.g_loc = [0.2, 5]

		self.r_pos = [0.2, 2.2]
		self.r_theta = 0.0

		self.des_twist = [0, 0, 0, 0, 0, 0]

		self.des_turn = 0
		self.des_speed = 0.5
		self.des_swait = 0

	def odom_callback(self, data):
	    # Your processing code here
	    # This function will be called whenever a new message is received
	    #print("pose: ", data.pose.pose)
	    #print("twist: ", data.twist.twist)
	    self.r_pos = [data.pose.pose.position.x, data.pose.pose.position.y]
	    self.r_theta = data.pose.pose.orientation.z
	    #print(data.pose.pose.position.x)

	def pose_callback(self, data):
	    # Your processing code here
	    # This function will be called whenever a new message is received
	    #print("pose:", data)
	    x = 0

	def straighten(self, des_or):
	    # Your processing code here
	    # This function will be called whenever a new message is received
	    #print("pose:", data)
	    #negative is right, pos is left
	    self.des_swait = 1
	    if self.des_swait >= 1:
	    	if self.r_theta - des_or > 0.01:
	    		if self.des_turn < 0.99:
	    			self.des_turn += 0.15 * abs(self.r_theta - des_or) #0.03 for 0.4
	    			self.des_swait = 0
	    			#print("desr", self.r_theta, des_or)
	    			if self.des_turn > 0.99:
	    				self.des_turn = 0.99
	    	elif self.r_theta - des_or < -0.01:
	    		if self.des_turn > -0.99:
		    		self.des_turn -= 0.15  * abs(self.r_theta - des_or)
		    		self.des_swait = 0
		    		#print("desl", self.r_theta, des_or)
		    		if self.des_turn < -0.99:
	    				self.des_turn = -0.99
	    	else:
	    		if self.des_turn > 0.05:
	    			self.des_turn -= abs(self.des_turn - 0.05) / 20 #10
	    			#print("correct_left: ", self.des_turn)
	    		if self.des_turn < 0.05:
	    			self.des_turn += abs(self.des_turn + 0.05) / 20
	    			#print("correct_right: ", self.des_turn)

	    else:
	    	self.des_swait += 0.5
	    	#print("des_wait")
	    

	def spin(self):
		rate = rospy.Rate(1000)
		# Publish the Joy message repeatedly
		while not rospy.is_shutdown():
			time_since_last_receive = rospy.Time.now() - self.last_received_time
			#print("time_gap: ", time_since_last_receive)

			if time_since_last_receive.to_sec() > 1.0 and time_since_last_receive.to_sec() < 2.0:
				# self.last_joy_message.buttons[r_btc["BTN_EAST"]] == 1
				self.joy_msg = Joy()
				self.joy_msg.axes = [0.0] * 8
				self.joy_msg.buttons = [0] * 12
				print("timeout")

			if (self.r_pos[0] < -1.5 or self.r_pos[0] > 1.5 or self.r_pos[1] > 6 or self.r_pos[1] < 1):
				self.joy_msg = Joy()
				self.joy_msg.axes = [0.0] * 8
				self.joy_msg.buttons = [0] * 12
				print("OB")
			elif (abs(self.r_pos[0] - self.g_loc[0]) > 0.1 or abs(self.r_pos[1] - self.g_loc[1]) > 0.1):
				self.last_received_time = rospy.Time.now()
				des_dir = 0
				y1 = self.r_pos[1] 
				y2 = self.g_loc[1]
				x1 = self.r_pos[0]
				x2 = self.g_loc[0]
				if x2 == x1:
					if y2 > y1:
						des_dir = 90.0
					else:
						des_dir = -90.0
				else:
					des_dir = math.atan2(self.g_loc[1] - self.r_pos[1], self.g_loc[0] - self.r_pos[0])
					des_dir = math.degrees(des_dir)
					des_dir = des_dir / 360
				print("angle: ", des_dir, self.r_theta, abs(self.r_theta))

				self.straighten(des_dir)
				self.joy_msg.axes[r_atc["ABS_RZ"]] = self.des_speed
				self.joy_msg.axes[r_atc["ABS_X"]] = self.des_turn
				#self.joy_msg.axes[r_atc["ABS_X"]] = 1.0
				#self.joy_msg.axes[r_atc["ABS_RX"]] = -1.0
				#print(self.r_pos, self.r_theta, self.des_turn)
			elif (abs(self.r_pos[0] - self.g_loc[0]) < 0.1 and abs(self.r_pos[1] - self.g_loc[1]) < 0.1):
				self.joy_msg = Joy()
				self.joy_msg.axes = [0.0] * 8
				self.joy_msg.buttons = [0] * 12
				print("SUCCESS")
			else:
				print("WTF", self.r_pos, self.g_loc)

			# Publish the Joy message
			if (time_since_last_receive.to_sec() < 2.0):
				self.joy_pub.publish(self.joy_msg)
			#print(joy_msg)

			# Sleep for a short time to avoid overwhelming the system
			rate.sleep()

if __name__ == '__main__':
	po_cont = POCont()
	po_cont.spin()
