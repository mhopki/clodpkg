#!/usr/bin/env python3

# Import required ROS and Python libraries
import rospy
from sensor_msgs.msg import Joy                # For publishing joystick messages
import inputs                                 # To handle input events (like joystick)
from nav_msgs.msg import Odometry             # For receiving odometry data
from geometry_msgs.msg import PoseStamped     # For handling pose data
import math                                   # Math operations
import random                                 # For generating random numbers
from std_msgs.msg import Float32MultiArray    # For multi-array messages
import numpy as np                            # For numerical operations
import argparse


# Initialize button and axis codes for joystick mappings
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

# Reverse mapping for button and axis codes
r_btc = {value: key for key, value in button_codes.items()}
r_atc = {value: key for key, value in axis_codes.items()}

# Main class for controlling the robot
class POCont:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('po_controller', anonymous=True)

        # Set up argument parser
        parser = argparse.ArgumentParser(description='ROS Node with arguments')
        parser.add_argument('--robot_number', type=int, required=True, help='The number of the robot')

        # Use parse_known_args() to ignore unrecognized ROS arguments
        args, unknown = parser.parse_known_args()

        self.robot_number = args.robot_number

        self.joy_pub_name = ['/joy','/joy1','/joy2','/joy3','/joy4']

        # Set up the publisher for joystick messages
        ##self.joy_pub = rospy.Publisher('/joy', Joy, queue_size=100)
        # Set up the publisher for joystick messages
        self.joy_pub = rospy.Publisher(self.joy_pub_name[self.robot_number], Joy, queue_size=100)

        self.odom_sub_name = ['/vicon/BEAST/odom','/vicon/BEAST_01/odom','/vicon/BEAST_02/odom','/vicon/BEAST_03/odom','/vicon/BEAST_04/odom']

        # Subscribe to the odometry topic for receiving odometry data
        ###self.odom_sub_topic = rospy.Subscriber('/vicon/BEAST_02/odom', Odometry, self.odom_callback, queue_size=1, tcp_nodelay=True)
        self.odom_sub_topic = rospy.Subscriber(self.odom_sub_name[self.robot_number], Odometry, self.odom_callback, queue_size=1, tcp_nodelay=True)

        # Subscribe to another odometry topic with a different callback
        #self.odom_sub = rospy.Subscriber(self.odom_sub_topic, Odometry, self.odom_callback_cam, queue_size=1, tcp_nodelay=True)

        # Subscribe to the waypoints topic to receive target waypoints
        #self.waypoint_sub = rospy.Subscriber('/waypoints', PoseStamped, self.waypoint_callback)

        self.waypoint_sub_name = ['/waypoints','/waypoints1','/waypoints2','/waypoints3','/waypoints4']

        # Subscribe to the waypoints topic to receive target waypoints
        self.waypoint_sub = rospy.Subscriber(self.waypoint_sub_name[self.robot_number], PoseStamped, self.waypoint_callback)

        # Initialize time, positions, and other variables
        self.last_received_time = rospy.Time.now()    # Last time a message was received
        self.current_waypoint = None                  # Current waypoint
        self.distance_threshold = 0.2                 # Distance threshold for goal detection

        # Initialize Joy message and other key variables
        self.joy_msg = Joy()                          # Joystick message template
        self.joy_msg.axes = [0.0] * 8                 # Initialize joystick axes
        self.joy_msg.buttons = [0] * 12               # Initialize joystick buttons
        self.last_joy_message = Joy()                 # Last joystick message sent
        self.r_odom = Odometry()                      # Odometry data for the robot
        self.g_odom = Odometry()                      # Goal odometry data
        self.g_met = False                            # Flag indicating if the goal is met

        # Initialize various position and motion control variables
        self.g_loc = [-10, -10]                       # Default goal location
        self.g_odom.pose.pose.position.x = self.g_loc[0]
        self.g_odom.pose.pose.position.y = self.g_loc[1]
        self.reset_point = 0                          # Counter for reset conditions

        self.waypoints = []                           # List of waypoints
        self.has_target = False                       # Flag for target presence
        self.targ_coords = [0.0, 0.0, 0.0]            # Target coordinates

        self.r_pos = [0.0, 0.0]                       # Robot position
        self.r_theta = -100.0                         # Robot orientation angle
        self.prev_theta = -100.0                      # Previous robot orientation
        self.r_theta_cam = [90, 0]                    # Camera orientation for the robot

        # Desired twist and other control parameters
        self.des_twist = [0, 0, 0, 0, 0, 0]           # Desired twist values
        self.hcommit = 1000                           # Heading commitment counter
        self.comside = 0                              # Commitment side indicator

        self.des_turn = 0                             # Desired turning angle
        self.des_speed = 0.5                          # Desired speed
        self.des_swait = 0                            # Speed waiting parameter
        self.g_thres = 0.3                           # Goal threshold
        self.targ_vel = 0.1

    # Callback for receiving odometry data
    def odom_callback(self, data):
        self.r_pos = [data.pose.pose.position.x, data.pose.pose.position.y]  # Update robot position

        # Initialize robot orientation if not set
        if self.r_theta == -100.0:
            self.r_theta = data.pose.pose.orientation.z
            self.prev_theta = self.r_theta

        self.r_twist = [data.twist.twist.linear.y, data.twist.twist.angular.z]  # Update twist (movement) data
        self.r_odom = data  # Save odometry data

    # Another callback for receiving odometry data with camera twist
    def odom_callback_cam(self, data):
        self.r_pos = [data.pose.pose.position.x, data.pose.pose.position.y]  # Update robot position

        # Initialize robot orientation if not set
        if self.r_theta == -100.0:
            self.r_theta = data.pose.pose.orientation.z
            self.prev_theta = self.r_theta

        self.r_twist = [data.twist.twist.linear.x, data.twist.twist.angular.z]  # Update twist (movement) data
        self.r_odom = data  # Save odometry data

    # Callback for receiving waypoints
    def waypoint_callback(self, data):
        print("way_coords[", self.robot_number, "]: ", data.pose.position.x, data.pose.position.y)  # Print waypoint coordinates
        self.waypoints.append(data)  # Add waypoint to the list

    # Pose callback (currently does nothing)
    def pose_callback(self, data):
        x = 0

    # Function to set the robot's orientation angle
    def set_rtheta(self):
        if abs(self.r_theta) < 99:  # Check if orientation is valid
            des_t = abs(self.r_odom.pose.pose.orientation.z)  # Desired orientation

            # Adjust orientation based on angular velocity and previous orientation
            if self.r_odom.twist.twist.angular.z > 0.05:
                if self.prev_theta > des_t:
                    self.prev_theta = des_t
                    self.r_theta = -des_t
                elif self.prev_theta < des_t:
                    self.prev_theta = des_t
                    self.r_theta = des_t
            elif self.r_odom.twist.twist.angular.z < -0.05:
                if self.prev_theta > des_t:
                    self.prev_theta = des_t
                    self.r_theta = des_t
                elif self.prev_theta < des_t:
                    self.prev_theta = des_t
                    self.r_theta = -des_t

    # Function to straighten the robot towards a desired orientation
    def straighten(self, des_or):
        self.des_swait = 1  # Set waiting state
        if self.des_swait >= 1:
            # Adjust turn based on current orientation difference
            if self.r_theta - des_or > 0.01:
                if self.des_turn < 0.99:
                    self.des_turn += 0.15 * abs(self.r_theta - des_or)  # Increase turn
                    self.des_swait = 0
                    if self.des_turn > 0.99:
                        self.des_turn = 0.99
            elif self.r_theta - des_or < -0.01:
                if self.des_turn > -0.99:
                    self.des_turn -= 0.15 * abs(self.r_theta - des_or)  # Decrease turn
                    self.des_swait = 0
                    if self.des_turn < -0.99:
                        self.des_turn = -0.99
            else:
                # Smooth turn adjustment if close to desired orientation
                if self.des_turn > 0.05:
                    self.des_turn -= abs(self.des_turn - 0.05) / 20
                if self.des_turn < 0.05:
                    self.des_turn += abs(self.des_turn + 0.05) / 20
        else:
            self.des_swait += 0.5

    # Calculate the distance between two points
    def calculate_distance(self, x1, y1, x2, y2):
        return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

    # Calculate the desired heading angle from the current position to the waypoint
    def calculate_desired_heading(self, current_pose, waypoint_pose):
        dx = waypoint_pose.pose.pose.position.x - current_pose.pose.pose.position.x  # Difference in x
        dy = waypoint_pose.pose.pose.position.y - current_pose.pose.pose.position.y  # Difference in y
        return math.atan2(-dx, dy)  # Calculate heading angle

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

    # Main update loop for the robot control
    def spin(self):
        # Set the loop rate to 1000 Hz
        rate = rospy.Rate(1000)

        # Initialize variables for scanning direction, linear output, and target velocity
        scan_dir = 0
        lin_out = 0
        #self.targ_vel = 0.1

        # Main loop that runs until ROS is shut down
        while not rospy.is_shutdown():
            # Calculate the time since the last joystick command was received
            time_since_last_receive = rospy.Time.now() - self.last_received_time

            # Set the robot's orientation using odometry data
            self.set_rtheta()  # Updates robot orientation based on current twist data
            fixed_odom = self.r_odom
            fixed_odom.pose.pose.orientation.z = self.r_theta  # Update the robot's orientation angle

            # Define motion gains for linear and angular movement
            lingain = 1.0 * 0.00065  # Linear gain factor
            anggain = 1.0 / (0.8 + ((fixed_odom.twist.twist.linear.y) / 0.5))#1.0 * 1.0 #2.0      # Angular gain factor

            # If no command has been sent recently (between 1 to 2 seconds), stop the robot
            if time_since_last_receive.to_sec() > 1.0 and time_since_last_receive.to_sec() < 2.0:
                # Send STOP commands to robot
                self.joy_msg = Joy()
                self.joy_msg.axes = [0.0] * 8
                self.joy_msg.buttons = [0] * 12

            # Check if the robot is out of bounds (currently disabled by False condition)
            if (False and (self.r_pos[0] < -9.0 or self.r_pos[0] > 9.0 or self.r_pos[1] > 9 or self.r_pos[1] < -2.0)):
                # Send STOP commands if the robot is out of bounds
                self.joy_msg = Joy()
                self.joy_msg.axes = [0.0] * 8
                self.joy_msg.buttons = [0] * 12
                print("OB")  # Print "Out of Bounds"

            # If the robot is traveling towards the goal but has not reached it
            elif ((abs(self.r_pos[0] - self.g_loc[0]) > self.g_thres or abs(self.r_pos[1] - self.g_loc[1]) > self.g_thres) 
                  and self.g_met == False and self.g_loc[0] != -10):
                # Calculate the distance to the goal
                dist = self.calculate_distance(self.r_pos[0], self.r_pos[1], self.g_loc[0], self.g_loc[1])

                # Calculate desired linear and angular commands based on gains and target velocity
                lin, ang = self.calculate_desired_cmd(fixed_odom, self.g_odom, lingain, anggain, self.targ_vel)

                # Cap the turn radius to prevent overly sharp turns
                if ang > 0.4:
                    ang = 0.4
                if ang < -0.4:
                    ang = -0.4

                # Soften the turn radius if close to the goal
                if dist < self.g_thres * 1.2:
                    print(dist)

                # Calculate desired linear velocity
                lin = self.calc_des_vel(fixed_odom, lingain, self.targ_vel)

                # Smooth adjustments for linear velocity if it's negative or positive
                if lin < 0:
                    lin /= 50
                    if lin < -0.01:
                        lin = -0.01
                if lin > 0:
                    if lin > 0.01:
                        lin = 0.01

                # Update linear output with constraints
                lin_out = lin_out + lin
                if lin_out < 0.1:
                    lin_out = 0.1
                if lin_out > 0.99:
                    lin_out = 0.99

                # Calculate forward and reverse angular outputs, capping them between -1.0 and 1.0
                ang_out_f = -((ang / 2) + 0.025) * 4
                if ang_out_f > 1.0:
                    ang_out_f = 1.0
                if ang_out_f < -1.0:
                    ang_out_f = -1.0
                ang_out_r = -((ang / 2)) * 4
                if ang_out_r > 1.0:
                    ang_out_r = 1.0
                if ang_out_r < -1.0:
                    ang_out_r = -1.0

                # Update the last received time to the current time
                self.last_received_time = rospy.Time.now()

                # Set joystick axes for driving based on retreating state
                self.joy_msg.axes[r_atc["ABS_Z"]] = 0        # No reverse
                self.joy_msg.axes[r_atc["ABS_RZ"]] = lin_out # Forward

                # Set joystick axes for turning
                self.joy_msg.axes[r_atc["ABS_X"]] = ang_out_f   # Forward angular adjustment
                self.joy_msg.axes[r_atc["ABS_RX"]] = -ang_out_r # Reverse angular adjustment

            # If the robot has reached the goal or the goal has been met
            elif ((abs(self.r_pos[0] - self.g_loc[0]) <= self.g_thres and abs(self.r_pos[1] - self.g_loc[1]) <= self.g_thres) 
                  or self.g_met == True) and self.g_loc[0] != -10:
                # Goal met; reset variables
                self.g_met = True
                self.reset_point += 1
                self.reset_point = 2000  # Reset point counter

                # If waiting at the reset point for 1000 steps
                if self.reset_point >= 1000:
                    # Check if all waypoints are completed
                    if len(self.waypoints) < 1:
                        print("complete")  # All waypoints completed
                        self.last_received_time = rospy.Time.now()
                        self.joy_msg.axes[r_atc["ABS_RZ"]] = 0
                        self.joy_msg.axes[r_atc["ABS_Z"]] = 0
                        self.joy_pub.publish(self.joy_msg)  # Publish stop command
                        self.g_loc = [-10, -10]             # Reset goal location
                        continue

                        # Generate new random goal location
                        nx = random.uniform(-1.0, 1.0)
                        ny = random.uniform(3.0, 6.0)
                        self.g_loc = [nx, ny]
                        self.g_odom.pose.pose.position.x = self.g_loc[0]
                        self.g_odom.pose.pose.position.y = self.g_loc[1]
                        self.reset_point = 0
                        self.g_met = False
                        print(self.g_loc)
                    else:
                        # Begin the next waypoint
                        self.g_loc = [self.waypoints[0].pose.position.x, self.waypoints[0].pose.position.y]
                        self.g_odom.pose.pose.position.x = self.g_loc[0]
                        self.g_odom.pose.pose.position.y = self.g_loc[1]
                        self.reset_point = 0
                        self.g_met = False
                        self.targ_vel = self.waypoints[0].pose.position.z
                        self.waypoints.pop(0)  # Remove the completed waypoint
                        print(self.g_loc)
            else:
                #print("IDLE ", self.robot_number)
                # If there are no waypoint commands, the robot is in an idle state
                if len(self.waypoints) > 0:
                    # Update goal location to current position
                    self.g_loc[0] = self.r_pos[0]
                    self.g_loc[1] = self.r_pos[1]

                # Example condition for testing (currently not in use)
                bag_testing = False
                if bag_testing:
                    # Calculate desired velocity with constraints
                    lin = self.calc_des_vel(fixed_odom, lingain, self.targ_vel)
                    if lin < 0:
                        lin /= 50
                    lin_out = lin_out + lin
                    if lin_out < 0.2:
                        lin_out = 0.2
                    if lin_out > 0.8:
                        lin_out = 0.8

                # Update the last received time
                self.last_received_time = rospy.Time.now()
                # Reset joystick axes (idle state)
                self.joy_msg.axes[r_atc["ABS_HAT0X"]] = 0
                self.joy_msg.axes[r_atc["ABS_HAT0Y"]] = 0
                self.joy_msg.axes[r_atc["ABS_X"]] = 0
                self.joy_msg.axes[r_atc["ABS_RX"]] = 0

            # Publish the joystick message if the time since last command is less than 2 seconds
            if time_since_last_receive.to_sec() < 2.0:
                self.joy_pub.publish(self.joy_msg)

            # Sleep for the defined rate to maintain loop timing
            rate.sleep()


# Main execution point
if __name__ == "__main__":
    try:
        po_controller = POCont()  # Create POCont object
        po_controller.spin()
    except rospy.ROSInterruptException:
        pass  # Handle ROS shutdown gracefully
