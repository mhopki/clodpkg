#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Joy
import inputs
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import math
import random
from std_msgs.msg import Float32MultiArray
import numpy as np


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
		self.odom_sub_topic = rospy.Subscriber('/vicon/BEAST/odom', Odometry, self.odom_callback, queue_size=1, tcp_nodelay=True)
		self.odom_sub = rospy.Subscriber(self.odom_sub_topic, Odometry, self.odom_callback_cam, queue_size=1, tcp_nodelay=True)
		self.waypoint_sub = rospy.Subscriber('/waypoints', PoseStamped, self.waypoint_callback)
		#pose_sub = rospy.Subscriber('/vicon/BEAST/pose', PoseStamped, pose_callback, queue_size=1, tcp_nodelay=True)

		self.last_received_time = rospy.Time.now()

		self.current_waypoint = None
		self.distance_threshold = 0.2

		self.joy_msg = Joy()
		self.joy_msg.axes = [0.0] * 8
		self.joy_msg.buttons = [0] * 12
		self.last_joy_message = Joy()
		self.r_odom = Odometry()
		self.g_odom = Odometry()
		self.g_met = False

		self.g_loc = [-10, -10]
		self.g_odom.pose.pose.position.x = self.g_loc[0]
		self.g_odom.pose.pose.position.y = self.g_loc[1]
		self.reset_point = 0

		self.waypoints = []
		self.has_target = False
		self.targ_coords = [0.0, 0.0, 0.0]

		self.r_pos = [0.0, 0.0]
		self.r_theta = -100.0
		self.prev_theta = -100.0
		self.r_theta_cam = [90,0]

		self.des_twist = [0, 0, 0, 0, 0, 0]
		self.hcommit = 1000
		self.comside = 0

		self.des_turn = 0
		self.des_speed = 0.5
		self.des_swait = 0
		self.g_thres = 0.3 #threshold of goal waypoint

		self.odom_switch = 12000#6000

		
	def odom_callback(self, data):

	    self.r_pos = [data.pose.pose.position.x, data.pose.pose.position.y]
	    if self.r_theta == -100.0:
	    	self.r_theta = data.pose.pose.orientation.z
	    	self.prev_theta = self.r_theta
	    self.r_twist = [data.twist.twist.linear.y, data.twist.twist.angular.z]
	    self.r_odom = data

	def odom_callback_cam(self, data):

	    self.r_pos = [data.pose.pose.position.x, data.pose.pose.position.y]
	    if self.r_theta == -100.0:
	    	self.r_theta = data.pose.pose.orientation.z
	    	self.prev_theta = self.r_theta
	    self.r_twist = [data.twist.twist.linear.x, data.twist.twist.angular.z]
	    self.r_odom = data

	def waypoint_callback(self, data):

	    print("way_coords: ", data.pose.position.x, data.pose.position.y)
	    self.waypoints.append(data)

	def pose_callback(self, data):
		#Do nothing
	    x = 0

	def set_rtheta(self):
		if abs(self.r_theta) < 99: #robot itself
			des_t = abs(self.r_odom.pose.pose.orientation.z)
			if self.r_odom.twist.twist.angular.z > 0.05:
				if self.prev_theta > des_t:
					self.prev_theta = des_t
					self.r_theta = -des_t
					#print("neg side")
				elif self.prev_theta < des_t:
					self.prev_theta = des_t
					self.r_theta = des_t
					#print("pos side")
			elif self.r_odom.twist.twist.angular.z < -0.05:
				if self.prev_theta > des_t:
					self.prev_theta = des_t
					self.r_theta = des_t
					#print("pos side")
				elif self.prev_theta < des_t:
					self.prev_theta = des_t
					self.r_theta = -des_t
					#print("neg side")


	def straighten(self, des_or):
	    #negative is right, pos is left
	    self.des_swait = 1
	    if self.des_swait >= 1:
	    	if self.r_theta - des_or > 0.01:
	    		if self.des_turn < 0.99:
	    			self.des_turn += 0.15 * abs(self.r_theta - des_or) #0.03 for 0.4
	    			self.des_swait = 0
	    			if self.des_turn > 0.99:
	    				self.des_turn = 0.99
	    	elif self.r_theta - des_or < -0.01:
	    		if self.des_turn > -0.99:
		    		self.des_turn -= 0.15  * abs(self.r_theta - des_or)
		    		self.des_swait = 0
		    		if self.des_turn < -0.99:
	    				self.des_turn = -0.99
	    	else:
	    		if self.des_turn > 0.05:
	    			self.des_turn -= abs(self.des_turn - 0.05) / 20
	    		if self.des_turn < 0.05:
	    			self.des_turn += abs(self.des_turn + 0.05) / 20

	    else:
	    	self.des_swait += 0.5

	def calculate_distance(self, x1, y1, x2, y2):
		return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

	def calculate_desired_heading(self, current_pose, waypoint_pose):
	    # Calculate the difference in x and y coordinates
	    dx = waypoint_pose.pose.pose.position.x - current_pose.pose.pose.position.x
	    dy = waypoint_pose.pose.pose.position.y - current_pose.pose.pose.position.y
	    
	    # Calculate the desired heading angle using arctangent (atan2)
	    desired_heading = math.atan2(dy, dx)
	    
	    return desired_heading

	def calculate_desired_cmd(self, current_pose, waypoint_pose, kp_linear, kp_angular, targ_vel):
        # Calculate desired heading
	    desired_heading = self.calculate_desired_heading(current_pose, waypoint_pose)
	    
	    # Calculate the difference between current and desired heading
	    heading_error = desired_heading - (current_pose.pose.pose.orientation.z * math.pi)
	    if (self.hcommit >= 10):#10000
	    	if heading_error > math.pi:
	    		heading_error -= 2 * math.pi
	    		self.comside = 1
	    		self.hcommit = 0
	    	elif heading_error < -math.pi:
	    		heading_error += 2 * math.pi
	    		self.comside = -1
	    		self.hcommit = 0
	    else:
	    	self.hcommit += 1
	    	if self.comside == 1:
	    		heading_error -= 2 * math.pi
	    	elif self.comside == -1:
	    		heading_error += 2 * math.pi
	    
	    # Calculate linear velocity as a proportion of the distance to the waypoint
	    linear_distance = math.sqrt((waypoint_pose.pose.pose.position.x - current_pose.pose.pose.position.x)**2 + 
	                                (waypoint_pose.pose.pose.position.y - current_pose.pose.pose.position.y)**2)

	    linear_distance = targ_vel - (abs(current_pose.twist.twist.linear.x) + abs(current_pose.twist.twist.linear.y) + abs(current_pose.twist.twist.linear.z))
	    linear_velocity = kp_linear * linear_distance
	    
	    # Calculate angular velocity as a proportion of the heading error
	    angular_velocity = kp_angular * heading_error

	    return linear_velocity, angular_velocity

	
	def calc_des_vel(self, current_pose, kp_linear, vel):
	    targ_vel = vel
	    linear_distance = targ_vel - (abs(current_pose.twist.twist.linear.x) + abs(current_pose.twist.twist.linear.y) + abs(current_pose.twist.twist.linear.z))
	    linear_velocity = kp_linear * linear_distance
	    return linear_velocity


	def angle(self, x1, y1, x2, y2):
		return math.atan2(y2 - y1, x2 - x1) 

	def calculate_point(self, x, y, distance, angle_radians):
		# Calculate the new point's coordinates
		new_x = x + distance * math.cos(angle_radians)
		new_y = y + distance * math.sin(angle_radians)

		return new_x, new_y

	def spin(self):
		rate = rospy.Rate(1000)
		# Publish the Joy message repeatedly
		scan_dir = 0
		lin_out = 0
		targ_vel = 0.1

		while not rospy.is_shutdown():
			time_since_last_receive = rospy.Time.now() - self.last_received_time

			#set robot orientation
			self.set_rtheta()
			fixed_odom = self.r_odom
			fixed_odom.pose.pose.orientation.z = self.r_theta #r_theta is robot orientation

			#motion gains
			lingain = 1.0 * 0.00065
			anggain = 1.0 * 2.0

			if time_since_last_receive.to_sec() > 1.0 and time_since_last_receive.to_sec() < 2.0:
				#NO COMMANDS SENT RECENTLY!!!
				#Send STOP commands to robot
				self.joy_msg = Joy()
				self.joy_msg.axes = [0.0] * 8
				self.joy_msg.buttons = [0] * 12
				x = 0

			if (False and (self.r_pos[0] < -9.0 or self.r_pos[0] > 9.0 or self.r_pos[1] > 9 or self.r_pos[1] < -2.0)):
				#Robot is out of bounds, send STOP commands to robot
				self.joy_msg = Joy()
				self.joy_msg.axes = [0.0] * 8
				self.joy_msg.buttons = [0] * 12
				print("OB")
				x = 0
			elif (((abs(self.r_pos[0] - self.g_loc[0]) > self.g_thres or abs(self.r_pos[1] - self.g_loc[1]) > self.g_thres) and self.g_met == False and self.g_loc[0] != -10)):
				#Robot is travelling

				dist = self.calculate_distance(self.r_pos[0], self.r_pos[1], self.g_loc[0], self.g_loc[1])

				lin, ang = self.calculate_desired_cmd(fixed_odom, self.g_odom, lingain, anggain, targ_vel)

				#fit desired turn radius
				if ang > 0.4:
					ang = 0.4
				if ang < -0.4:
					ang = -0.4

				#soften turn radius if close to goal
				if dist < self.g_thres * 1.2:
					print(dist)

				lin = self.calc_des_vel(fixed_odom, lingain, targ_vel)
				if lin < 0:
					lin /= 50
					if lin < -0.01:
						lin = -0.01
				if lin > 0:
					if lin > 0.01:
						lin = 0.01
				lin_out = lin_out + lin
				if lin_out < 0.65:
					lin_out = 0.65
				if lin_out > 1.0:
					lin_out = 1.0

				ang_out_f = -((ang / 2) + 0.025)*4
				if ang_out_f > 1.0:
					ang_out_f = 1.0
				if ang_out_f < -1.0:
					ang_out_f = -1.0
				ang_out_r = -((ang / 2))*4
				if ang_out_r > 1.0:
					ang_out_r = 1.0
				if ang_out_r < -1.0:
					ang_out_r = -1.0

				self.last_received_time = rospy.Time.now()
				if (self.retreating == 2):
					self.joy_msg.axes[r_atc["ABS_Z"]] = lin_out
					self.joy_msg.axes[r_atc["ABS_RZ"]] = 0
				else:
					self.joy_msg.axes[r_atc["ABS_Z"]] = 0
					self.joy_msg.axes[r_atc["ABS_RZ"]] = lin_out
				self.joy_msg.axes[r_atc["ABS_X"]] = ang_out_f
				self.joy_msg.axes[r_atc["ABS_RX"]] = -ang_out_r

			elif (((abs(self.r_pos[0] - self.g_loc[0]) <= self.g_thres and abs(self.r_pos[1] - self.g_loc[1]) <= self.g_thres) or self.g_met == True) and self.g_loc[0] != -10):
				#Reached goal
				self.g_met == True

				self.reset_point += 1
				self.reset_point = 2000

				#If waited at reset point for 1000 steps
				if self.reset_point >= 1000:
					#if completed all way points
					if len(self.waypoints) < 1:
						print("complete")
						self.last_received_time = rospy.Time.now()
						self.joy_msg.axes[r_atc["ABS_RZ"]] = 0
						self.joy_msg.axes[r_atc["ABS_Z"]] = 0
						self.joy_pub.publish(self.joy_msg)
						self.g_loc = [-10, -10]
						continue
						nx = random.uniform(-1.0, 1.0)
						ny = random.uniform(3.0, 6.0)
						self.g_loc = [nx, ny]
						self.g_odom.pose.pose.position.x = self.g_loc[0]
						self.g_odom.pose.pose.position.y = self.g_loc[1]
						self.reset_point = 0
						self.g_met = False
						print(self.g_loc)
					else:
						#begin next waypoint
						self.g_loc = [self.waypoints[0].pose.position.x, self.waypoints[0].pose.position.y]
						self.g_odom.pose.pose.position.x = self.g_loc[0]
						self.g_odom.pose.pose.position.y = self.g_loc[1]
						self.reset_point = 0
						self.g_met = False
						self.waypoints.pop(0)
						print(self.g_loc)
			else:
				x = 0
				#No waypoint commands, Idle state

				if len(self.waypoints) > 0:
					self.g_loc[0] = self.r_pos[0]
					self.g_loc[1] = self.r_pos[1]

				bag_testing = False
				if (bag_testing):
					lin = self.calc_des_vel(fixed_odom, lingain, targ_vel)
					if lin < 0:
						lin /= 50
					lin_out = lin_out + lin
					if lin_out < 0.2:
						lin_out = 0.2
					if lin_out > 0.8:
						lin_out = 0.8


				self.last_received_time = rospy.Time.now()
				#self.joy_msg.axes[r_atc["ABS_RZ"]] = lin_out#if bag_testing
				self.joy_msg.axes[r_atc["ABS_HAT0X"]] = 0
				self.joy_msg.axes[r_atc["ABS_HAT0Y"]] = 0
				self.joy_msg.axes[r_atc["ABS_X"]] = 0
				self.joy_msg.axes[r_atc["ABS_RX"]] = 0

			# Publish the Joy message
			if (time_since_last_receive.to_sec() < 2.0):
				x = 0
				self.joy_pub.publish(self.joy_msg)

			# Sleep for a short time to avoid overwhelming the system
			rate.sleep()

if __name__ == '__main__':
	po_cont = POCont()
	po_cont.spin()
