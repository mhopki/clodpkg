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
		rospy.init_node('po_controller', anonymous=True) #SET QUEUESIZE TO 1

		#Specific odom topic robot subscribes to
		self.odom_sub_topic = '/camera/odom/sample'#'/odometry/filtered_map'#'/camera/odom/sample'#'fused_localization'#'/camera/odom/sample'

		#List of odom topics (for switching between VIO, GPS, etc)
		self.odom_topics = ['/camera/odom/sample','/odometry/filtered_map', '/camera/odom/sample', 'fused_localization']

		#Subscriber for Robot control commands (JOY commands)
		self.joy_pub = rospy.Publisher('/joy', Joy, queue_size=1) #delete queue_size = 1

		#Subsriber for robot odometry
		self.odom_sub = rospy.Subscriber(self.odom_sub_topic, Odometry, self.odom_callback, queue_size=1, tcp_nodelay=True)

		#Subscriber for receiving waypoints to navigate to
		self.waypoint_sub = rospy.Subscriber('/waypoints', PoseStamped, self.waypoint_callback)

		#Subscriber for the target coordinates that the RGB camera should face
		self.target_sub = rospy.Subscriber('/object_world_coordinates', Float32MultiArray, self.world_coordinates_callback)

		#Publisher publishes a specific number that the Hyperspectral code receives to know to take a picture
		self.photo_pub = rospy.Publisher('/take_photo', Float32MultiArray, queue_size=1)
		#Subscriber listens to a specific number that the lets the robot know the picture has been taken and to continue following its path
		self.photo_sub = rospy.Subscriber('/take_photo', Float32MultiArray, self.photo_callback)

		self.last_received_time = rospy.Time.now()


		#Initialize an empty JOY command that will tell the robot to idle
		self.joy_msg = Joy()
		self.joy_msg.axes = [0.0] * 8
		self.joy_msg.buttons = [0] * 12
		self.last_joy_message = Joy()


		self.r_odom = Odometry() #Robot Odometry
		self.g_odom = Odometry() #Goal Odometry (just created so i can easily compare the two)
		self.g_met = False #If current goal location is met

		self.g_loc = [-10, -10] #Goal location array[x,y] (-10, -10 is the intialized state so that robot doesnt think it has reached a goal when it turns on)
		self.g_odom.pose.pose.position.x = self.g_loc[0]
		self.g_odom.pose.pose.position.y = self.g_loc[1]
		self.reset_point = 0 #conunter for when the next waypoint is set to the new goal after the old goal is met

		self.waypoints = []
		self.has_target = False #if there is a target to track with the RGB camera
		self.targ_coords = [0.0, 0.0, 0.0] #target coordinates
		self.cam_pose = [0,0] #RGB cam pose x,y

		self.r_pos = [0.0, 0.0] #Robot position array
		self.r_theta = -100.0 #Special theta for orientation (wrong in principle, but it works correctly) it is updated each step but initialized to -100
		self.prev_theta = -100.0 #Previous Special theta
		self.r_theta_cam = [90,0] #Angle of the RGB camera (x,y)

		self.hcommit = 1000 #Variable for deiding which turn to take when reorienting. Make sure robot understands +350deg is also -10deg and to take the latter
		self.comside = 0 #side that robot chooses to turn, either "+350deg" or "-10deg"

		self.g_thres = 0.3 #threshold of goal waypoint

		self.odom_switch = 12000#6000 #Time to wait before switching Odom topic. Used to start in VIO and the switch to GPS once a couple messages have been received to give the GPS the proper orientation

		self.HOLDON = 0 #pause for the given time to wait and see if there are YOLO detections
		self.HOLDON_wait = 0 #wait up time before you can pause again for YOLO detections again 
		self.HOLDON_hits = 0 #YOLO sensor detections during pause
		self.sensor_locs = [] #loc of sensor confirmed
		self.sensor_locs_my = [] #my location when i saw confirmed sensor
		self.retreating = 0 #retreating to find sensor, prevent seeing another "holding on"
		self.reverse = 0 #Makes ROBOT REVERSE
		self.RETREATPIC = 0 # pause for hyperspectral photo
		self.wcoord = [] #world coordinates of current sensor detection
		self.wcoord_all = [] #world coordinate of all sensor detections

		self.photo_good = False #Did the hyperspectral successfully take a picture

	def world_coordinates_callback(self, data):
		#YOLO/REDTAPE detection received

	    if self.HOLDON: #count the number of detections in the HOLDON period
		    self.HOLDON_hits += 1
		    print("hit!")

	    if self.HOLDON_wait <= 0 and self.retreating <= 0 and self.RETREATPIC <=0: #Starts a HOLDON period which causes the robot to stop and wait to see if it gets more detections
		    self.HOLDON = 2000
		    self.HOLDON_wait = 2000
		    print("HOLDON!!!!")

	    if self.RETREATPIC > 0: #if takin hyperspectral picture, save the estimate world coorinates of the YOLO/REDTAPE
	    	self.wcoord = data.data

	    #N I G H T M A R E
	    #Make the TARGET coordinates, that the camera faces, hone in on the YOLO/REDTAPE world coordinates, so that the YOLO/REDTAPE stays in the center of the image
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
		#Receive Odometry
	    self.r_pos = [data.pose.pose.position.x, data.pose.pose.position.y]
	    if self.r_theta == -100.0:
	    	self.r_theta = data.pose.pose.orientation.z
	    	self.prev_theta = self.r_theta
	    self.r_odom = data

	def waypoint_callback(self, data):
		#Receive published waypoint
	    print("way_coords: ", data.pose.position.x, data.pose.position.y)
	    self.waypoints.append(data)

	def photo_callback(self, data):
		#If received message that Hyperspectral has successfully taken a photo
	    if data.data[0] == 1:
	        self.photo_good = True

	def cam_orient(self, current_pose, ox, oy, oz, kpx, kpy, kpz):
		#N I G H T M A R E
		#XY plane
	    dx = ox - current_pose.pose.pose.position.x
	    dy = oy - current_pose.pose.pose.position.y
	    desired_heading = math.atan2(-dy, -dx) - (math.pi*0.5)#-dy, -dx what is reverse

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
	    	rev_off = (truecy - 2.0)
	    	truecy = 3.0 - rev_off #FOR FLIPPED CAM
			#print("1 zone")
	    elif 1.0 - (heading_error_gen / math.pi) != 1.0 :
	    	#print("overextend")
	    	truecy = (abs(abs((3.0 - (heading_error_gen / math.pi) + (self.cam_pose[1] - 2.0)) - 3.0) + 2.0) - 3.0) + 2.0
	    	#print("DESIRED: ", truecy)
	    	#truecy = 2.0#-(1.0 + (heading_error_gen / math.pi) / 5) + (self.cam_pose[1])
	    	#truecy = -(1.0 + (heading_error_gen / math.pi) / 5) + (self.cam_pose[1])
	    	#truecy = 2.0 + (heading_error_gen / math.pi)#+ (1.0 + (heading_error_gen / math.pi))
	    	if truecy < 2.0: truecy = 2.0
	    	#truecy = 3.0
	    	rev_off = (truecy-2.0)
	    	truecy = 3.0 - rev_off #FOR FLIPPED CAM
	    	#print("2 zone")

	    cey = ((kpy * 0)/math.pi + 3.0)

	    return truecx, truecy

	def set_rtheta(self):
		#Converts orientation Z into a 0 to 2pi scale (incorrect in principle, but works correctly)
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


	def calculate_distance(self, x1, y1, x2, y2):
		return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

	def calculate_desired_heading(self, current_pose, waypoint_pose):
	    # Calculate the difference in x and y coordinates
	    dx = waypoint_pose.pose.pose.position.x - current_pose.pose.pose.position.x
	    dy = waypoint_pose.pose.pose.position.y - current_pose.pose.pose.position.y
	    
	    # Calculate the desired heading angle using arctangent (atan2)
	    if (self.retreating == 2 or self.reverse): #REVERSE
	    	desired_heading = -math.atan2(dy, dx)#-math.atan2(dx, dy)
	    else: #FORWARD
	    	desired_heading = math.atan2(dy, dx)#-math.atan2(dx, dy)
	    
	    return desired_heading

	def calculate_desired_cmd(self, current_pose, waypoint_pose, kp_linear, kp_angular, targ_vel):
        # Calculate desired heading
	    desired_heading = self.calculate_desired_heading(current_pose, waypoint_pose)
	    #targ_vel = 0.4
	    
	    # Calculate the difference between current and desired heading
	    heading_error = desired_heading - (current_pose.pose.pose.orientation.z * math.pi)
	    #Commit to turning to one side or the other
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

	    #linear_distance = targ_vel - (abs(current_pose.twist.twist.linear.x) + abs(current_pose.twist.twist.linear.y) + abs(current_pose.twist.twist.linear.z))
	    velocity_mag = targ_vel - math.sqrt(current_pose.twist.twist.linear.x ** 2 + current_pose.twist.twist.linear.y ** 2 + current_pose.twist.twist.linear.z ** 2)
	    linear_velocity = kp_linear * velocity_mag
	    
	    # Calculate angular velocity as a proportion of the heading error
	    angular_velocity = kp_angular * heading_error

	    return linear_velocity, angular_velocity

	
	def calc_des_vel(self, current_pose, kp_linear, vel):
	    targ_vel = vel
	    #linear_distance = targ_vel - (abs(current_pose.twist.twist.linear.x) + abs(current_pose.twist.twist.linear.y) + abs(current_pose.twist.twist.linear.z))
	    velocity_mag = targ_vel - math.sqrt(current_pose.twist.twist.linear.x ** 2 + current_pose.twist.twist.linear.y ** 2 + current_pose.twist.twist.linear.z ** 2)
	    linear_velocity = kp_linear * velocity_mag
	    return linear_velocity

	def cam_track(self, fixed_odom, kpx, kpy, kpz):
		if True:
			if True:
				#No waypoint commands, Idle state
				#print("waiting")
				if (True):
					tracking_test = True
					# Assuming you have the camera pitch angle in radians
					theta_pitch =  -math.pi + (self.cam_pose[1] - 2.0 * (math.pi))

					# Assuming you have the camera position in the world frame
					camera_position = [fixed_odom.pose.pose.position.x, fixed_odom.pose.pose.position.y, fixed_odom.pose.pose.position.z]

					# Object coordinates in the camera frame
					object_coordinates_cam = [(self.targ_coords[0]/100), (self.targ_coords[1]/100)*4.0, -(self.targ_coords[2]/100)] #MADE Z NEGATIVE AFTER FLIPPING CAM AROUND

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
						divnum = 500#100
						sidenum = 1.0#2.0#4.0
						dumx = fixed_odom.pose.pose.position.x - (self.targ_coords[0]/divnum) #FLIPPED CAMERA MAKES IT A MINUS SIGN   #object_coordinates_world[0] #fixed_odom.pose.pose.position.x + object_coordinates_world[0]
						dumy = fixed_odom.pose.pose.position.y + (self.targ_coords[1]/divnum)*sidenum# object_coordinates_world[1] #fixed_odom.pose.pose.position.y + object_coordinates_world[1]
						dumz = object_coordinates_world[2] #fixed_odom.pose.pose.position.z + object_coordinates_world[2]
						if self.devx == None:
							self.devx = dumx
							self.devy = dumy
							self.devz = dumz
						else:
							deviant = (abs(self.devx - dumx) + abs(self.devx - dumx) + abs(self.devx - dumx))/3
							#print("DEVIANT: ", deviant, self.devx, dumx)
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
									"""
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
									"""
									#self.waypoints.append(eyeg)
									self.persist = 100
								else:
									self.persist -= 1
							else:
								self.persist = 100

					else:
						#BASIC
						dumx = fixed_odom.pose.pose.position.x# + 10
						dumy = fixed_odom.pose.pose.position.y# + 10
						dumz = fixed_odom.pose.pose.position.z# + 0
					
					print("dumyCORDS: ", [dumx, dumy, dumz])

					camx, camy = self.cam_orient(fixed_odom, dumx, dumy, dumz, kpx, kpy, kpz)
					print("camx:", camx, ", camy:", camy )
					#camx = 2.5
					#camy = 2.0
					self.cam_pose = [camx, camy]

					return camx, camy, dumx, dumy, dumz

	def angle(self, x1, y1, x2, y2):
		return math.atan2(y2 - y1, x2 - x1) 

	def calculate_point(self, x, y, distance, angle_radians):
		# Calculate the new point's coordinates
		new_x = x + distance * math.cos(angle_radians)
		new_y = y + distance * math.sin(angle_radians)

		return new_x, new_y

	def spin(self):
		rate = rospy.Rate(1000) #Rate of Joy commands, if you decrease this the robot will have a time delay
		# intialize variables
		scan_dir = 0
		lin_out = 0
		targ_vel = 0.1 #CONSTANT VELOCITY of the robot

		while not rospy.is_shutdown():
			time_since_last_receive = rospy.Time.now() - self.last_received_time

			if self.HOLDON <= 0 and self.HOLDON_wait > 0:#Runs out the wait period needed before stopping again when a sensor is detected
				self.HOLDON_wait -= 1

			#set robot orientation
			self.set_rtheta()
			fixed_odom = self.r_odom
			fixed_odom.pose.pose.orientation.z = self.r_theta #r_theta is robot orientation

			#motion gains
			lingain = 1.0 * 0.0002 #VELOCITY GAIN (may need to play with this if robot does not accelerate fast enough)
			anggain = 1.0 * 2.0 #ANGULAR VELOCITY GAIN, (you shouldnt have to touch this one)
			
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

			if (False and (self.r_pos[0] < -9.0 or self.r_pos[0] > 9.0 or self.r_pos[1] > 9 or self.r_pos[1] < -2.0)):
				#-3.0, 3.0, 9, 0.5
				#OUTTA BOUNDS!!!
				#Robot is out of bounds, send STOP commands to robot
				self.joy_msg = Joy()
				self.joy_msg.axes = [0.0] * 8
				self.joy_msg.buttons = [0] * 12
				print("OB")
				x = 0
			elif (self.HOLDON <= 0 and self.RETREATPIC <= 0 and ((abs(self.r_pos[0] - self.g_loc[0]) > self.g_thres or abs(self.r_pos[1] - self.g_loc[1]) > self.g_thres) and self.g_met == False and self.g_loc[0] != -10)):
				#WE ON GO
				#Robot is travelling to waypoint
				if self.odom_switch > 0:
					if self.odom_sub_topic == self.odom_topics[0]:
						self.odom_switch -= 1
						#print("still odom")
						if self.odom_switch <= 0: #switches from one odom topic to the next in the list MAY BE NECESSARY TO USE COMMENTED CODE BELOW
							xxxx= 0
							#print("switch to fused")
							#self.odom_sub_topic = self.odom_topics[1]
							#self.odom_sub.unregister()
							#self.odom_sub = rospy.Subscriber(self.odom_sub_topic, Odometry, self.odom_callback, queue_size=1, tcp_nodelay=True)

				dist = self.calculate_distance(self.r_pos[0], self.r_pos[1], self.g_loc[0], self.g_loc[1])

				lin, ang = self.calculate_desired_cmd(fixed_odom, self.g_odom, lingain, anggain, targ_vel)
				scan_dir = 0
				if (self.has_target or True):
					#SET CAMERA TO FIXED HOMOGRAPHY ORIENTATION
					camx = 2.507#2.25
					camy = 3.0#2.0#2.1
					if self.retreating > 0:
						camx = 2.507
						camy = 3.0#2.0
					#self.cam_track(fixed_odom, kpx, kpy, kpz)
					#camx, camy = self.cam_orient(fixed_odom, self.g_loc[0], self.g_loc[1], fixed_odom.pose.pose.position.z + 0, kpx, kpy, kpz)
				else:
					dumx = fixed_odom.pose.pose.position.x + 10*math.cos(scan_dir + self.r_theta)
					dumy = fixed_odom.pose.pose.position.y + 10*math.sin(scan_dir + self.r_theta)
					camx, camy = self.cam_orient(fixed_odom, dumx, dumy, fixed_odom.pose.pose.position.z + 0, kpx, kpy, kpz)
					scan_dir += 0.001
					if scan_dir > math.pi:
						scan_dir -= 2 * math.pi
					elif scan_dir < -math.pi:
						scan_dir += 2 * math.pi

				#send camera orientation joy commands
				self.joy_msg.axes[r_atc["ABS_HAT0X"]] = camx
				self.joy_msg.axes[r_atc["ABS_HAT0Y"]] = camy

				#fit desired turn radius so servos dont turn to far and shatter
				if ang > 0.4:
					ang = 0.4
				if ang < -0.4:
					ang = -0.4


				lin = self.calc_des_vel(fixed_odom, lingain, targ_vel)
				#How largely the throttle command decreases based on how far the velocity is above the desired
				if lin < 0:
					lin /= 50 #decrease or increase if desired to make robot decelerate faster
					if lin < -0.01: #0.01:
						lin = -0.01
				#How largely the throttle command increases based on how far the velocity is above the desired
				if lin > 0:
					lin /= 1 #decrease or increase if desired to make robot accelerate faster
					if lin > 0.01: #0.01:
						lin = 0.01
				lin_out = lin_out + lin # + 0.2

				#Throttle command bottom limit
				#MAY NEED TO INCREASE WHEN OUTSIDE
				if lin_out < 0.1: #15#0.85
					lin_out = 0.1 #15#0.85

				#Throttle command upper limit
				if lin_out > 1.0:
					lin_out = 1.0

				#Turning angle command
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
				if (self.retreating == 2 or self.reverse): #REVERSE DRIVING
					self.joy_msg.axes[r_atc["ABS_Z"]] = lin_out
					self.joy_msg.axes[r_atc["ABS_RZ"]] = 0
				else: #FORWARD DRIVING
					self.joy_msg.axes[r_atc["ABS_Z"]] = 0
					self.joy_msg.axes[r_atc["ABS_RZ"]] = lin_out
				self.joy_msg.axes[r_atc["ABS_X"]] = ang_out_f
				self.joy_msg.axes[r_atc["ABS_RX"]] = -ang_out_r
			elif (((abs(self.r_pos[0] - self.g_loc[0]) <= self.g_thres and abs(self.r_pos[1] - self.g_loc[1]) <= self.g_thres) or self.g_met == True) and self.g_loc[0] != -10):
				#Made it to the goal
				self.g_met == True

				#print("SUCCESS")
				self.reset_point += 1
				self.reset_point = 2000 #Instantly go to next waypoint

				#If waited at reset point for 1000 steps
				if self.reset_point >= 1000: 
					#if completed all way points
					if len(self.waypoints) < 1:
						print("complete")
						#Send stop joy command and reset goal location to nothing
						self.last_received_time = rospy.Time.now()
						self.joy_msg.axes[r_atc["ABS_RZ"]] = 0
						self.joy_msg.axes[r_atc["ABS_Z"]] = 0
						self.joy_pub.publish(self.joy_msg)
						self.g_loc = [-10, -10]
						continue
					else:
						#begin next waypoint
						self.g_loc = [self.waypoints[0].pose.position.x, self.waypoints[0].pose.position.y]
						self.g_odom.pose.pose.position.x = self.g_loc[0]
						self.g_odom.pose.pose.position.y = self.g_loc[1]
						self.reset_point = 0
						self.g_met = False
						self.waypoints.pop(0)
						print(self.g_loc)

			elif self.HOLDON:
				print("HOLDON")
				camx, camy, senx, seny, senz = self.cam_track(fixed_odom, kpx, kpy, kpz)

				#set camera to fixed homography position
				camx = 2.507#2.5
				camy = 3.0
				if self.HOLDON <= 1000:
					#set camera to fixed homography position
					camx = 2.507#2.5
					camy = 3.0#2.0


				self.HOLDON -= 1
				if self.HOLDON == 0:
					self.HOLDON_wait = 1000#5000 #how long to wait for 5 detections
					if self.HOLDON_hits >= 5: #5 YOLO detections found during wait period means there is a sensor so take a pic
						print("FOUND A SENSOR!!")
						print("FOUND A SENSOR!!")
						print("FOUND A SENSOR!!")
						print("FOUND A SENSOR!!")

						#self.sensor_locs.append([senx, seny, senz]) #OBSOLETE needs to be fixed
						#print("SENSOR LOC: ", self.sensor_locs) #OBSOLETE needs to be fixed
						self.sensor_locs_my.append([fixed_odom.pose.pose.position.x, fixed_odom.pose.pose.position.y, fixed_odom.pose.pose.position.z])
						print("MY LOC: ", self.sensor_locs_my)
						
						#dist = self.calculate_distance(self.sensor_locs[-1][0], self.sensor_locs[-1][1], self.sensor_locs_my[-1][0], self.sensor_locs_my[-1][1])

						if (True):
							#Start taking Hyperspectral pic and send photo message to the Hyperspectral
							photo_msg = Float32MultiArray(data=[0])
							self.photo_pub.publish(photo_msg)
							self.RETREATPIC = 1000#4000
							self.photo_good = False
							print("WE FLICKING UP FR!!!!!!!!")

					self.HOLDON_hits = 0

				self.last_received_time = rospy.Time.now()

				#STOP ROBOT and control cam orientation
				self.joy_msg.axes[r_atc["ABS_HAT0X"]] = camx
				self.joy_msg.axes[r_atc["ABS_HAT0Y"]] = camy
				if self.joy_msg.axes[r_atc["ABS_RZ"]] > 0:
					self.joy_msg.axes[r_atc["ABS_RZ"]] = 0
				if self.joy_msg.axes[r_atc["ABS_RZ"]] <= 0.01:
					self.joy_msg.axes[r_atc["ABS_RZ"]] = 0
				if self.joy_msg.axes[r_atc["ABS_Z"]] > 0:
					self.joy_msg.axes[r_atc["ABS_Z"]] = 0
				if self.joy_msg.axes[r_atc["ABS_Z"]] <= 0.01:
					self.joy_msg.axes[r_atc["ABS_Z"]] = 0

			elif self.RETREATPIC:
				#Taking hyperspectral photo, and waiting for message that picture was successful

				if self.photo_good == True: #if successful picture, leave this loop
					self.RETREATPIC -= 1

				if self.RETREATPIC == 0:
					print("COORDINATES: ", self.wcoord)
					self.wcoord_all.append(self.wcoord)
					self.HOLDON_wait = 2000
					self.photo_good = False
					#x = self.wcoor[25]

				self.last_received_time = rospy.Time.now()

				#SLOW DOWN ROBOT WHETHER ITS GOING BACKWARD OR FORWARD
				if self.joy_msg.axes[r_atc["ABS_RZ"]] > 0:
					self.joy_msg.axes[r_atc["ABS_RZ"]] -= 0.01
				if self.joy_msg.axes[r_atc["ABS_RZ"]] <= 0.01:
					self.joy_msg.axes[r_atc["ABS_RZ"]] = 0
				if self.joy_msg.axes[r_atc["ABS_Z"]] > 0:
					self.joy_msg.axes[r_atc["ABS_Z"]] -= 0.01
				if self.joy_msg.axes[r_atc["ABS_Z"]] <= 0.01:
					self.joy_msg.axes[r_atc["ABS_Z"]] = 0
			else:
				x = 0
				#No waypoint commands, Idle state

				#print("waiting")
				#print("sensors_found: ", self.sensor_locs)
				#print("my_location_then: ", self.sensor_locs_my)

				if (self.has_target or True):
					camx, camy, _, _, _ = self.cam_track(fixed_odom, kpx, kpy, kpz)

					#camx = 2.5
					#camy = 3.0#2.0

					#Set camera to fixed orientation (Homography was done in this fixed orientation)
					camx = 2.507
					camy = 3.0
				
				else:
					dumx = fixed_odom.pose.pose.position.x + 10*math.cos(scan_dir)
					dumy = fixed_odom.pose.pose.position.y + 10*math.sin(scan_dir)
					camx, camy = self.cam_orient(fixed_odom, fixed_odom.pose.pose.position.x + (dumx/100), fixed_odom.pose.pose.position.y + (dumy/100), fixed_odom.pose.pose.position.z + 0, kpx, kpy, kpz)
					scan_dir += 0.001
					if scan_dir > math.pi:
						scan_dir -= 2 * math.pi
					elif scan_dir < -math.pi:
						scan_dir += 2 * math.pi

				if len(self.waypoints) > 0: #CHECK IF WAYPOINTS EXIST TO START FOLLOWING
					self.g_loc[0] = self.r_pos[0]
					self.g_loc[1] = self.r_pos[1]


				self.last_received_time = rospy.Time.now()
				#self.joy_msg.axes[r_atc["ABS_RZ"]] = lin_out#if bag_testing
				self.joy_msg.axes[r_atc["ABS_HAT0X"]] = camx
				self.joy_msg.axes[r_atc["ABS_HAT0Y"]] = camy
				self.joy_msg.axes[r_atc["ABS_X"]] = 0#ang_out_f
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
