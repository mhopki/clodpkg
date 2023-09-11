#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Joy
import inputs
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import math
import random


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
		#self.odom_sub = rospy.Subscriber('/vicon/BEAST/odom', Odometry, self.odom_callback, queue_size=1, tcp_nodelay=True)
		self.odom_sub = rospy.Subscriber('/camera/odom/sample', Odometry, self.odom_callback_cam, queue_size=1, tcp_nodelay=True)
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

	def odom_callback(self, data):
	    # Your processing code here
	    # This function will be called whenever a new message is received
	    #print("pose: ", data.pose.pose)
	    #print("twist: ", data.twist.twist)
	    self.r_pos = [data.pose.pose.position.x, data.pose.pose.position.y]
	    if self.r_theta == -100.0:
	    	self.r_theta = data.pose.pose.orientation.z
	    	self.prev_theta = self.r_theta
	    self.r_twist = [data.twist.twist.linear.y, data.twist.twist.angular.z]
	    self.r_odom = data
	    #print(self.r_odom)
	    #print(data.pose.pose.position.x)
	    #print(self.r_odom.pose.pose.orientation.z, self.r_odom.twist.twist.angular.z, self.r_theta)

	def odom_callback_cam(self, data):
	    # Your processing code here
	    # This function will be called whenever a new message is received
	    #print("pose: ", data.pose.pose)
	    #print("twist: ", data.twist.twist)
	    self.r_pos = [data.pose.pose.position.x, data.pose.pose.position.y]
	    if self.r_theta == -100.0:
	    	self.r_theta = data.pose.pose.orientation.z
	    	self.prev_theta = self.r_theta
	    self.r_twist = [data.twist.twist.linear.x, data.twist.twist.angular.z]
	    self.r_odom = data
	    #print(self.r_odom)
	    #print(data.pose.pose.position.x)
	    #print(self.r_odom.pose.pose.orientation.z, self.r_odom.twist.twist.angular.z, self.r_theta)

	def waypoint_callback(self, data):
	    # Your processing code here
	    # This function will be called whenever a new message is received
	    #self.g_loc = [self.g_odom.pose.pose.position.x, self.g_odom.pose.pose.position.y]
	    #self.g_odom = data
	    #print("got waypoint")
	    print("way_coords: ", data.pose.position.x, data.pose.position.y)
	    self.waypoints.append(data)

	def pose_callback(self, data):
	    # Your processing code here
	    # This function will be called whenever a new message is received
	    #print("pose:", data)
	    x = 0

	def cam_orient(self, current_pose, ox, oy, kpx, kpy):
	    dx = ox - current_pose.pose.pose.position.x
	    dy = oy - current_pose.pose.pose.position.y
	    desired_heading = math.atan2(dy, -dx) - (math.pi*0.5)

	    cur_or = current_pose.pose.pose.orientation.z 
	    if cur_or < 0:
	    	xx = 0
	    	#also alter the z direction

	    heading_error_gen = desired_heading - (cur_or * math.pi) - (math.radians(180))
	    heading_error = desired_heading - (cur_or * math.pi) - (math.radians(self.r_theta_cam[0]))
	    #print("ORIG cam_error: ", heading_error, heading_error_gen)
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
	    print("cam_error: ", heading_error, heading_error_gen)
	    cex = ((kpx * heading_error)/math.pi + 3.0)
	    if (1.0 - (heading_error_gen / math.pi) > 0 and 1.0 - (heading_error_gen / math.pi) <= 1.0):
	    	truecx = 3.0 - (heading_error_gen / math.pi)
	    else:
	    	truecx = 3.0 - (1.0 + (heading_error_gen / math.pi))
	    cey = ((kpy * 0)/math.pi + 3.0)
	    #truecx = (cex - 3.0)
	    if heading_error_gen > 0:
	    	truecy = 2.0
	    else:
	    	truecy = 2.0
	    print("cmds: ", cex, cey)
	    print("cmds_true: ", truecx, truecy)
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
			#self.r_theta += self.r_odom.twist.twist.angular.z
			#if self.r_theta < -1:
			#	self.r_theta = 1
			#if self.r_theta > 1:
			#	self.r_theta = -1


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

	def calculate_distance(self, x1, y1, x2, y2):
		return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

	def calculate_desired_heading(self, current_pose, waypoint_pose):
	    # Calculate the difference in x and y coordinates
	    dx = waypoint_pose.pose.pose.position.x - current_pose.pose.pose.position.x
	    dy = waypoint_pose.pose.pose.position.y - current_pose.pose.pose.position.y
	    
	    # Calculate the desired heading angle using arctangent (atan2)
	    desired_heading = math.atan2(dy, dx)#-math.atan2(dx, dy)

	    # Normalize the desired heading to the range of 0 to 2π
	    # Normalize the desired heading to the range of -π to π
	    #if desired_heading > math.pi:
	    #    desired_heading -= 2 * math.pi
	    #elif desired_heading < -math.pi:
	    #    desired_heading += 2 * math.pi
	    
	    return desired_heading

	def calculate_desired_cmd(self, current_pose, waypoint_pose, kp_linear, kp_angular):
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
	    linear_velocity = kp_linear * linear_distance
	    
	    # Calculate angular velocity as a proportion of the heading error
	    angular_velocity = kp_angular * heading_error

	    #print("theta error: ", heading_error)
	    #print("lin: ", linear_velocity, "| ang: ", angular_velocity)
	    #linear_velocity = 0.4
	    #print(desired_heading, " vs ", current_pose.pose.pose.orientation.z, current_pose.pose.pose.orientation.z * math.pi)
	    #print(desired_heading, " vs ", current_pose.pose.pose.orientation.z)
	    #print(math.degrees(desired_heading), " vs ", math.degrees(current_pose.pose.pose.orientation.z * math.pi) )

	    return linear_velocity, angular_velocity


	"""
	def calculate_desired_angular_acceleration(self, current_pose, waypoint_pose, max_angular_acceleration):
	    # Calculate the desired heading angle
	    desired_heading = self.calculate_desired_heading(current_pose, waypoint_pose)
	    
	    # Calculate the difference between current and desired heading angles
	    current_heading = self.calculate_heading_from_quaternion(current_pose.orientation)
	    heading_difference = self.angle_difference(desired_heading, current_heading)
	    
	    # Calculate desired angular accelerationheading_error
	    desired_angular_acceleration = max(min(heading_difference, max_angular_acceleration),
	                                        -max_angular_acceleration)
	    
	    return desired_angular_acceleration
	"""
	    

	def spin(self):
		rate = rospy.Rate(1000)
		# Publish the Joy message repeatedly
		while not rospy.is_shutdown():
			time_since_last_receive = rospy.Time.now() - self.last_received_time
			self.set_rtheta()
			fixed_odom = self.r_odom
			fixed_odom.pose.pose.orientation.z = self.r_theta
			lingain = 3.0 * 6#3.0#1.5
			anggain = 8.0 * 20#8.0#5.0#3.0
			kpx = 1.0
			kpy = 1.0
			#print("time_gap: ", time_since_last_receive)

			if time_since_last_receive.to_sec() > 1.0 and time_since_last_receive.to_sec() < 2.0:
				# self.last_joy_message.buttons[r_btc["BTN_EAST"]] == 1
				#NO COMMANDS SENT RECENTLY!!!
				self.joy_msg = Joy()
				self.joy_msg.axes = [0.0] * 8
				self.joy_msg.buttons = [0] * 12
				#print("timeout")
				x = 0

			if (self.r_pos[0] < -9.0 or self.r_pos[0] > 9.0 or self.r_pos[1] > 9 or self.r_pos[1] < -2.0):
				#-3.0, 3.0, 9, 0.5
				#OUTTA BOUNDS!!!
				self.joy_msg = Joy()
				self.joy_msg.axes = [0.0] * 8
				self.joy_msg.buttons = [0] * 12
				print("OB")
				x = 0
			elif ((abs(self.r_pos[0] - self.g_loc[0]) > self.g_thres or abs(self.r_pos[1] - self.g_loc[1]) > self.g_thres) and self.g_met == False and self.g_loc[0] != -10):
				#WE ON GO
				dist = self.calculate_distance(self.r_pos[0], self.r_pos[1], self.g_loc[0], self.g_loc[1])
				#fixed_odom = self.r_odom
				#fixed_odom.pose.pose.orientation.z = self.r_theta
				lin, ang = self.calculate_desired_cmd(fixed_odom, self.g_odom, lingain, anggain)
				cam_dir = self.cam_orient(fixed_odom, 5, 0)
				#print("outs:", lin, ang)

				if lin > 0.4:
					lin = 0.4

				if ang > 0.4:
					ang = 0.4
				if ang < -0.4:
					ang = -0.4
				if dist < self.g_thres * 1.2:
					#ang = ang / 10
					print(dist)

				lin_out = lin # + 0.2
				if lin_out < 0.2:
					lin_out = 0.2
				lin_out = 0.5
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
				#self.joy_msg.axes[r_atc["ABS_RZ"]] = lin_out
				self.joy_msg.axes[r_atc["ABS_X"]] = ang_out_f
				self.joy_msg.axes[r_atc["ABS_RX"]] = -ang_out_r
				"""
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
				self.joy_msg.axes[r_atc["ABS_X"]] = self.des_turn"""
				#self.joy_msg.axes[r_atc["ABS_X"]] = 1.0
				#self.joy_msg.axes[r_atc["ABS_RX"]] = -1.0
				#print(self.r_pos, self.r_theta, self.des_turn)
			elif (((abs(self.r_pos[0] - self.g_loc[0]) <= self.g_thres and abs(self.r_pos[1] - self.g_loc[1]) <= self.g_thres) or self.g_met == True) and self.g_loc[0] != -10):
				#WE MADE IT NIGGA!!!
				self.g_met == True
				#self.joy_msg = Joy()
				#self.joy_msg.axes = [0.0] * 8
				#self.joy_msg.buttons = [0] * 12
				#print("SUCCESS")
				self.reset_point += 1
				self.reset_point = 2000
				#print(self.reset_point)
				if self.reset_point >= 1000:
					if len(self.waypoints) < 1:
						print("complete")
						self.last_received_time = rospy.Time.now()
						self.joy_msg.axes[r_atc["ABS_RZ"]] = 0
						self.joy_pub.publish(self.joy_msg)
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
						self.g_loc = [self.waypoints[0].pose.position.x, self.waypoints[0].pose.position.y]
						self.g_odom.pose.pose.position.x = self.g_loc[0]
						self.g_odom.pose.pose.position.y = self.g_loc[1]
						self.reset_point = 0
						self.g_met = False
						self.waypoints.pop(0)
						print(self.g_loc)
			else:
				x = 0
				print("wtf")
				camx, camy = self.cam_orient(fixed_odom, 5, 0, kpx, kpy)
				print("cur_pos: ", self.r_theta)
				if len(self.waypoints) > 0:
					self.g_loc[0] = self.r_pos[0]
					self.g_loc[1] = self.r_pos[1]
				#print("WTF", self.r_pos, self.g_loc)
				self.last_received_time = rospy.Time.now()
				#self.joy_msg.axes[r_atc["ABS_RZ"]] = lin_out
				self.joy_msg.axes[r_atc["ABS_HAT0X"]] = 2.0
				#self.joy_msg.axes[r_atc["ABS_HAT0Y"]] = 0

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
