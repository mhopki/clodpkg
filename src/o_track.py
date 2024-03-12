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
		self.odom_sub_topic = '/camera/odom/sample'#'fused_localization'#'/camera/odom/sample'
		self.joy_pub = rospy.Publisher('/joy', Joy, queue_size=100)
		#self.odom_sub = rospy.Subscriber('/vicon/BEAST/odom', Odometry, self.odom_callback, queue_size=1, tcp_nodelay=True)
		self.odom_sub = rospy.Subscriber(self.odom_sub_topic, Odometry, self.odom_callback_cam, queue_size=1, tcp_nodelay=True)
		self.waypoint_sub = rospy.Subscriber('/waypoints', PoseStamped, self.waypoint_callback)
		#pose_sub = rospy.Subscriber('/vicon/BEAST/pose', PoseStamped, pose_callback, queue_size=1, tcp_nodelay=True)
		self.target_sub = rospy.Subscriber('/object_world_coordinates', Float32MultiArray, self.world_coordinates_callback)

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
		self.cam_pose = [0,0]

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

		#BASE VALUES FOR TRACKING TEST!!!
		self.devx = None
		self.devy = None
		self.devz = None
		self.persist = 100

	def world_coordinates_callback(self, data):
	    x_world, y_world, z_world = data.data
	    dx = x_world - self.targ_coords[0]
	    dy = y_world - self.targ_coords[1]
	    dz = z_world - self.targ_coords[2]
	    dx_inc = (((dx**2)*(dx/abs(dx))) * 0.02)
	    dy_inc = (((dy**2)*(dy/abs(dy))) * 0.02)
	    dz_inc = (((dz**2)*(dz/abs(dz))) * 0.01)
	    dx_max = 2.0
	    dy_max = 2.0
	    dz_max = 1.0#half

	    dx_min = 0.001
	    dy_min = 0.001
	    dz_min = 0.001
	    if dx_inc < -dx_max: dx_inc = -dx_max
	    if dx_inc > dx_max: dx_inc = dx_max
	    if dy_inc < -dy_max: dy_inc = -dy_max
	    if dy_inc > dy_max: dy_inc = dy_max
	    if dz_inc < -dz_max: dz_inc = -dz_max
	    if dz_inc > dz_max: dz_inc = dz_max

	    if dx_inc < 0 and dx_inc > -dx_min: dx_inc = -dx_min
	    if dx_inc > 0 and dx_inc < dx_min: dx_inc = dx_min
	    if dy_inc < 0 and dy_inc > -dy_min: dy_inc = -dy_min
	    if dy_inc > 0 and dy_inc < dy_min: dy_inc = dy_min
	    if dz_inc < 0 and dz_inc > -dz_min: dz_inc = -dz_min
	    if dz_inc > 0 and dz_inc < dz_min: dz_inc = dz_min

	    self.targ_coords[0] = self.targ_coords[0] + dx_inc
	    self.targ_coords[1] = self.targ_coords[1] + dy_inc
	    self.targ_coords[2] = self.targ_coords[2] + dz_inc
	    rospy.loginfo(f"Received World Coordinates: X={x_world:.2f} cm, Y={y_world:.2f} cm, Z={z_world:.2f} cm")
		
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

	def cam_orient(self, current_pose, ox, oy, oz, kpx, kpy, kpz):
		#XY plane
	    dx = ox - current_pose.pose.pose.position.x
	    dy = oy - current_pose.pose.pose.position.y
	    desired_heading = math.atan2(-dy, -dx) - (math.pi*0.5)#-dy, -dx

	    #print("dx, dy, des_heading: ", dx, dy, desired_heading, math.atan2(-dy, -dx))

	    cur_or = current_pose.pose.pose.orientation.z 
	    if cur_or < 0:
	    	xx = 0
	    	#also alter the z direction
	    #print("cur_or: ", cur_or)

	    #print("hd_comp: ", desired_heading, (cur_or * math.pi), (math.radians(180)))

	    heading_error_gen = desired_heading - (cur_or * math.pi) - (math.radians(180))
	    heading_error = desired_heading - (cur_or * math.pi) - (math.radians(self.r_theta_cam[0]))
	    
	    #print("head_er_gem, head_er: ", heading_error_gen, heading_error)

	    #Correct orientation of robot to fit within -pi to pi
	    if (True):#10000
	    	if heading_error > math.pi:
	    		heading_error -= 2 * math.pi
	    	elif heading_error < -math.pi:
	    		heading_error += 2 * math.pi

	    if (True):#10000
	    	if heading_error_gen > math.pi:
	    		heading_error_gen -= 2 * math.pi
	    	if heading_error_gen < -math.pi:
	    		heading_error_gen += 2 * math.pi
	    if (True):#10000
	    	if heading_error_gen > math.pi:
	    		heading_error_gen -= 2 * math.pi
	    	if heading_error_gen < -math.pi:
	    		heading_error_gen += 2 * math.pi
	    
	    heading_error = -heading_error
	    heading_error_gen = -heading_error_gen

	    #print("FIXED: head_er_gem, head_er: ", heading_error_gen, heading_error)
	    
	    cex = ((kpx * heading_error)/math.pi + 3.0)
	    if (1.0 - (heading_error_gen / math.pi) > 0 and 1.0 - (heading_error_gen / math.pi) <= 1.0):
	    	truecx = 3.0 - (heading_error_gen / math.pi)
	    	truecy = 2.0
	    else:
	    	truecx = 3.0 - (1.0 + (heading_error_gen / math.pi))
	    	truecy = 3.0
	    cey = ((kpy * 0)/math.pi + 3.0)


	    #dz = oz - current_pose.pose.pose.position.z
	    #truecy = 2.0 + 0.5*(dz/30)
	    #if truecy < 2.0: truecy = 2.0

	    #XZ plane
	    dx = ox - current_pose.pose.pose.position.x
	    dz = oz - current_pose.pose.pose.position.z
	    desired_heading = math.atan2(dy, -dz) - (math.pi*0.5)#-dy, -dx

	    #print("dx, dz, des_heading: ", dx, dz, desired_heading, math.atan2(-dz, -dx))

	    cur_or = ((self.cam_pose[1] - 2.0)) #current_pose.pose.pose.orientation.z 
	    if cur_or < 0:
	    	xx = 0
	    	#also alter the z direction
	    #print("cur_or: ", cur_or)

	    #print("hd_comp: ", desired_heading, (cur_or * math.pi), (math.radians(180)))

	    heading_error_gen = desired_heading - (cur_or * math.pi) - (math.radians(180))
	    heading_error = desired_heading - (cur_or * math.pi) - (math.radians(self.r_theta_cam[0]))
	    
	    #print("head_er_gem, head_er: ", heading_error_gen, heading_error)

	    #Correct orientation of robot to fit within -pi to pi
	    if (True):#10000
	    	if heading_error > math.pi:
	    		heading_error -= 2 * math.pi
	    	elif heading_error < -math.pi:
	    		heading_error += 2 * math.pi

	    if (True):#10000
	    	if heading_error_gen > math.pi:
	    		heading_error_gen -= 2 * math.pi
	    	if heading_error_gen < -math.pi:
	    		heading_error_gen += 2 * math.pi
	    if (True):#10000
	    	if heading_error_gen > math.pi:
	    		heading_error_gen -= 2 * math.pi
	    	if heading_error_gen < -math.pi:
	    		heading_error_gen += 2 * math.pi
	    
	    heading_error = -heading_error
	    heading_error_gen = -heading_error_gen

	    #print("FIXED: head_er_gem, head_er: ", heading_error_gen, heading_error)
	    
	    cex = ((kpx * heading_error)/math.pi + 3.0)
	    if (1.0 - (heading_error_gen / math.pi) > 0 and 1.0 - (heading_error_gen / math.pi) <= 1.0):
	    	truecy = abs((3.0 - (heading_error_gen / math.pi) + (self.cam_pose[1] - 2.0)) - 3.0) + 2.0
	    	#truecy = 2.0 + (heading_error_gen / math.pi)
	    	#if truecy < 2.0: truecy = 2.0
	    	#truecy = 2.0
	    elif 1.0 - (heading_error_gen / math.pi) != 1.0 :
	    	#print("overextend")
	    	truecy = (abs(abs((3.0 - (heading_error_gen / math.pi) + (self.cam_pose[1] - 2.0)) - 3.0) + 2.0) - 3.0) + 2.0
	    	#print("DESIRED: ", truecy)
	    	#truecy = 2.0#-(1.0 + (heading_error_gen / math.pi) / 5) + (self.cam_pose[1])
	    	#truecy = -(1.0 + (heading_error_gen / math.pi) / 5) + (self.cam_pose[1])
	    	#truecy = 2.0 + (heading_error_gen / math.pi)#+ (1.0 + (heading_error_gen / math.pi))
	    	if truecy < 2.0: truecy = 2.0
	    	#truecy = 3.0
	    cey = ((kpy * 0)/math.pi + 3.0)

	    return truecx, truecy

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

	def calculate_distance(self, x1, y1, x2, y2):
		return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

	def calculate_desired_heading(self, current_pose, waypoint_pose):
	    # Calculate the difference in x and y coordinates
	    dx = waypoint_pose.pose.pose.position.x - current_pose.pose.pose.position.x
	    dy = waypoint_pose.pose.pose.position.y - current_pose.pose.pose.position.y
	    
	    # Calculate the desired heading angle using arctangent (atan2)
	    desired_heading = math.atan2(dy, dx)#-math.atan2(dx, dy)
	    
	    return desired_heading

	def calculate_desired_cmd(self, current_pose, waypoint_pose, kp_linear, kp_angular, targ_vel):
        # Calculate desired heading
	    desired_heading = self.calculate_desired_heading(current_pose, waypoint_pose)
	    #targ_vel = 0.4
	    
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
	    

	def spin(self):
		rate = rospy.Rate(1000)
		# Publish the Joy message repeatedly
		scan_dir = 0
		lin_out = 0
		targ_vel = 0.4

		while not rospy.is_shutdown():
			time_since_last_receive = rospy.Time.now() - self.last_received_time

			#set robot orientation
			self.set_rtheta()
			fixed_odom = self.r_odom
			fixed_odom.pose.pose.orientation.z = self.r_theta #r_theta is robot orientation

			#motion gains
			lingain = 1.0 * 0.002#0.01#3.0 * 6#3.0#1.5
			anggain = 8.0 * 20#8.0#5.0#3.0
			
			#cam heading gains
			kpx = 1.0
			kpy = 1.0
			kpz = 1.0
			#print("time_gap: ", time_since_last_receive)

			if time_since_last_receive.to_sec() > 1.0 and time_since_last_receive.to_sec() < 2.0:
				# self.last_joy_message.buttons[r_btc["BTN_EAST"]] == 1
				#NO COMMANDS SENT RECENTLY!!!
				#Send STOP commands to robot
				self.joy_msg = Joy()
				self.joy_msg.axes = [0.0] * 8
				self.joy_msg.buttons = [0] * 12
				#print("timeout")
				x = 0

			if (self.r_pos[0] < -9.0 or self.r_pos[0] > 9.0 or self.r_pos[1] > 9 or self.r_pos[1] < -2.0):
				#-3.0, 3.0, 9, 0.5
				#OUTTA BOUNDS!!!
				#Robot is out of bounds, send STOP commands to robot
				self.joy_msg = Joy()
				self.joy_msg.axes = [0.0] * 8
				self.joy_msg.buttons = [0] * 12
				print("OB")
				x = 0
			elif ((abs(self.r_pos[0] - self.g_loc[0]) > self.g_thres or abs(self.r_pos[1] - self.g_loc[1]) > self.g_thres) and self.g_met == False and self.g_loc[0] != -10):
				#WE ON GO
				#Robot is travelling
				dist = self.calculate_distance(self.r_pos[0], self.r_pos[1], self.g_loc[0], self.g_loc[1])

				lin, ang = self.calculate_desired_cmd(fixed_odom, self.g_odom, lingain, anggain, targ_vel)
				scan_dir = 0
				if (self.has_target or True):
					camx, camy = self.cam_orient(fixed_odom, self.g_loc[0], self.g_loc[1], fixed_odom.pose.pose.position.z + 0, kpx, kpy, kpz)
				else:
					dumx = fixed_odom.pose.pose.position.x + 10*math.cos(scan_dir + self.r_theta)
					dumy = fixed_odom.pose.pose.position.y + 10*math.sin(scan_dir + self.r_theta)
					camx, camy = self.cam_orient(fixed_odom, dumx, dumy, fixed_odom.pose.pose.position.z + 0, kpx, kpy, kpz)
					scan_dir += 0.001
					if scan_dir > math.pi:
						scan_dir -= 2 * math.pi
					elif scan_dir < -math.pi:
						scan_dir += 2 * math.pi

				self.joy_msg.axes[r_atc["ABS_HAT0X"]] = camx
				self.joy_msg.axes[r_atc["ABS_HAT0Y"]] = camy

				#fit desired turn radius
				if ang > 0.4:
					ang = 0.4
				if ang < -0.4:
					ang = -0.4

				#soften turn radius if close to goal
				if dist < self.g_thres * 1.2:
					#ang = ang / 10
					print(dist)

				lin = self.calc_des_vel(fixed_odom, lingain, targ_vel)
				if lin < 0:
					lin /= 50
					if lin < -0.01:
						lin = -0.01
				if lin > 0:
					if lin > 0.01:
						lin = 0.01
				lin_out = lin_out + lin # + 0.2
				if lin_out < 0.2:
					lin_out = 0.2
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
				self.joy_msg.axes[r_atc["ABS_RZ"]] = lin_out
				self.joy_msg.axes[r_atc["ABS_X"]] = ang_out_f
				self.joy_msg.axes[r_atc["ABS_RX"]] = -ang_out_r
			elif (((abs(self.r_pos[0] - self.g_loc[0]) <= self.g_thres and abs(self.r_pos[1] - self.g_loc[1]) <= self.g_thres) or self.g_met == True) and self.g_loc[0] != -10):
				#WE MADE IT NIGGA!!!
				self.g_met == True

				#print("SUCCESS")
				self.reset_point += 1
				self.reset_point = 2000

				#If waited at reset point for 1000 steps
				if self.reset_point >= 1000:
					#if completed all way points
					if len(self.waypoints) < 1:
						print("complete")
						self.last_received_time = rospy.Time.now()
						self.joy_msg.axes[r_atc["ABS_RZ"]] = 0
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
				print("waiting")
				if (self.has_target or True):
					tracking_test = True
					# Assuming you have the camera pitch angle in radians
					theta_pitch =  -math.pi + (self.cam_pose[1] - 2.0 * (math.pi))

					# Assuming you have the camera position in the world frame
					camera_position = [fixed_odom.pose.pose.position.x, fixed_odom.pose.pose.position.y, fixed_odom.pose.pose.position.z]

					# Object coordinates in the camera frame
					object_coordinates_cam = [(self.targ_coords[0]/100), (self.targ_coords[1]/100)*4.0, (self.targ_coords[2]/100)]

					# Define the rotation matrix for pitch
					"""
					R_pitch = np.array([[1, 0, 0],
					                    [0, np.cos(theta_pitch), -np.sin(theta_pitch)],
					                    [0, np.sin(theta_pitch), np.cos(theta_pitch)]])
					"""
					"""
					R_pitch = np.array([[np.cos(theta_pitch), 0, np.sin(theta_pitch)],
					                    [0, 1, 0],
					                    [-np.sin(theta_pitch), 0, np.cos(theta_pitch)]])
					"""

					R_pitch = np.array([[np.cos(theta_pitch), -np.sin(theta_pitch), 0],
					                    [np.sin(theta_pitch), np.cos(theta_pitch), 0],
					                    [0, 0, 1]])
					

					# Transform object coordinates to the world frame
					object_coordinates_world = np.dot(R_pitch, object_coordinates_cam)

					# Adjust for camera position
					object_coordinates_world += camera_position
					#print("theta, cam_pos: ", theta_pitch, camera_position)
					#print("obj_cam: ", object_coordinates_cam)
					#print("obj_world: ", object_coordinates_world)

					if (tracking_test == True):
						dumx = fixed_odom.pose.pose.position.x + (self.targ_coords[0]/100) #object_coordinates_world[0] #fixed_odom.pose.pose.position.x + object_coordinates_world[0]
						dumy = fixed_odom.pose.pose.position.y + (self.targ_coords[1]/100)*4.0# object_coordinates_world[1] #fixed_odom.pose.pose.position.y + object_coordinates_world[1]
						dumz = object_coordinates_world[2] #fixed_odom.pose.pose.position.z + object_coordinates_world[2]
						if self.devx == None:
							self.devx = dumx
							self.devy = dumy
							self.devz = dumz
						else:
							deviant = (abs(self.devx - dumx) + abs(self.devx - dumx) + abs(self.devx - dumx))/3
							print("DEVIANT: ", deviant, self.devx, dumx)
							self.devx = dumx
							self.devy = dumy
							self.devz = dumz
							if (deviant < 0.5):
								if self.persist <= 0:
									eyeg = []
									eyeg = PoseStamped()
									direction_x = fixed_odom.pose.pose.position.x - (fixed_odom.pose.pose.position.x + (dumx/100))
									direction_y = fixed_odom.pose.pose.position.y - (fixed_odom.pose.pose.position.x + (dumy/100))
									distance = math.sqrt(direction_x ** 2 + direction_y ** 2)
									if distance != 0:
										direction_x /= distance
										direction_y /= distance
									eyeg.pose.position.x = fixed_odom.pose.pose.position.x + (dumx/100) + ((dumz/100) / 2.0) * direction_x
									eyeg.pose.position.y = (fixed_odom.pose.pose.position.y + (dumy/100) + ((dumz/100) / 2.0) * direction_y)
									print("EYE: ", eyeg.pose.position.x, eyeg.pose.position.y)
									print("EYE: ", eyeg.pose.position.x, eyeg.pose.position.y)
									print("EYE: ", eyeg.pose.position.x, eyeg.pose.position.y)
									print("EYE: ", eyeg.pose.position.x, eyeg.pose.position.y)
									print("EYE: ", eyeg.pose.position.x, eyeg.pose.position.y)
									print("EYE: ", eyeg.pose.position.x, eyeg.pose.position.y)
									print("EYE: ", eyeg.pose.position.x, eyeg.pose.position.y)
									print("EYE: ", eyeg.pose.position.x, eyeg.pose.position.y)
									print("EYE: ", eyeg.pose.position.x, eyeg.pose.position.y)
									print("EYE: ", eyeg.pose.position.x, eyeg.pose.position.y)
									print("EYE: ", eyeg.pose.position.x, eyeg.pose.position.y)
									print("EYE: ", eyeg.pose.position.x, eyeg.pose.position.y)
									#self.waypoints.append(eyeg)
									self.persist = 100
								else:
									self.persist -= 1
							else:
								self.persist = 100

					else:
						#BASIC
						dumx = fixed_odom.pose.pose.position.x + 10
						dumy = fixed_odom.pose.pose.position.y + 10
						dumz = fixed_odom.pose.pose.position.z + 0
					
					print("dumyCORDS: ", [dumx, dumy, dumz])

					camx, camy = self.cam_orient(fixed_odom, dumx, dumy, dumz, kpx, kpy, kpz)
					print("camx:", camx, ", camy:", camy )
					#camx = 2.5
					#camy = 2.0
					self.cam_pose = [camx, camy]
				
				else:
					dumx = fixed_odom.pose.pose.position.x + 10*math.cos(scan_dir)
					dumy = fixed_odom.pose.pose.position.y + 10*math.sin(scan_dir)
					camx, camy = self.cam_orient(fixed_odom, fixed_odom.pose.pose.position.x + (dumx/100), fixed_odom.pose.pose.position.y + (dumy/100), fixed_odom.pose.pose.position.z + 0, kpx, kpy, kpz)
					scan_dir += 0.001
					if scan_dir > math.pi:
						scan_dir -= 2 * math.pi
					elif scan_dir < -math.pi:
						scan_dir += 2 * math.pi
					#print("dummy_dir: ", dumx, dumy, scan_dir)
				#camx, camy = self.cam_orient(fixed_odom, 0, 1, kpx, kpy)
				#print("cur_pos: ", self.r_theta)
				if len(self.waypoints) > 0:
					self.g_loc[0] = self.r_pos[0]
					self.g_loc[1] = self.r_pos[1]
				#print("WTF", self.r_pos, self.g_loc)

				bag_testing = False
				if (bag_testing):
					#print("s: ", lin_out)
					lin = self.calc_des_vel(fixed_odom, lingain, targ_vel)
					if lin < 0:
						lin /= 50
					lin_out = lin_out + lin # + 0.2
					if lin_out < 0.2:
						lin_out = 0.2
					if lin_out > 0.8:
						lin_out = 0.8
					#print("u: ", lin_out, lin)


				self.last_received_time = rospy.Time.now()
				#self.joy_msg.axes[r_atc["ABS_RZ"]] = lin_out#if bag_testing
				self.joy_msg.axes[r_atc["ABS_HAT0X"]] = camx
				self.joy_msg.axes[r_atc["ABS_HAT0Y"]] = camy
				self.joy_msg.axes[r_atc["ABS_X"]] = 0.1#ang_out_f
				self.joy_msg.axes[r_atc["ABS_RX"]] = 0#-ang_out_r

			# Publish the Joy message
			if (time_since_last_receive.to_sec() < 2.0):
				x = 0
				self.joy_pub.publish(self.joy_msg)
			#print(joy_msg)

			# Sleep for a short time to avoid overwhelming the system
			rate.sleep()

if __name__ == '__main__':
	po_cont = POCont()
	po_cont.spin()
