#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Joy
import inputs


# Initialize the ROS node and publisher
rospy.init_node('gamepad_publisher', anonymous=True)
joy_pub = rospy.Publisher('/joy', Joy, queue_size=100)

# Create a Joy message
joy_msg = Joy()
joy_msg.axes = [0.0] * 8
joy_msg.buttons = [0] * 12

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

last_joy_message = Joy()

rate = rospy.Rate(1000)

# Publish the Joy message repeatedly
while not rospy.is_shutdown():
    events = inputs.get_gamepad()
    for event in events:
        # Update the Joy message based on the event
        if event.ev_type == 'Key':
        	btn_idx = r_btc[event.code]
        	joy_msg.buttons[btn_idx] = event.state
        	#print("okay")
        	#print(event)
        	#print(event.code)
        	#print(event.state)
        elif event.ev_type == 'Absolute':
        	axs_idx = r_atc[event.code]
        	if (axs_idx == 0 or axs_idx == 1 or axs_idx == 2 or axs_idx == 3):
        		joy_msg.axes[axs_idx] = event.state / 32767.0
        		if abs(joy_msg.axes[axs_idx]) < 0.1:
        			joy_msg.axes[axs_idx] = 0
        		elif joy_msg.axes[axs_idx] < -1.0:
        			joy_msg.axes[axs_idx] = -1.0
        	elif (axs_idx == 6 or axs_idx == 7):
        		joy_msg.axes[axs_idx] = event.state / 255.0
        	else:
        		joy_msg.axes[axs_idx] = event.state
       			#print("Event Output: ", event.state)

    # Publish the Joy message
    joy_pub.publish(joy_msg)
    #print(joy_msg)

    # Sleep for a short time to avoid overwhelming the system
    rate.sleep()
