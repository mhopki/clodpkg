#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Joy

import time
from adafruit_servokit import ServoKit

#import Jetson.GPIO as GPIO
import board
import digitalio

# Set channels to the number of servo channels on your kit.
# 8 for FeatherWing, 16 for Shield/HAT/Bonnet.
kit = ServoKit(channels=16)

# Pin Definitions
#led = digitalio.DigitalInOut(board.D18)
#led.direction = digitalio.Direction.OUTPUT
motor_pin_a = digitalio.DigitalInOut(board.D27)#13  # BOARD pin 12, BCM pin 18
motor_pin_a.direction = digitalio.Direction.OUTPUT
motor_pin_b = digitalio.DigitalInOut(board.D18)#12  # BOARD pin 16, BCM pin 16
motor_pin_b.direction = digitalio.Direction.OUTPUT

# motor_pin_a.value = False
# motor_pin_b.value = False

# there are 12 indices in the /joy/button, but only 10 here, and only '0' is used.
button_codes = {
    0: "BTN_EAST", # red (east), seems to be the breaking button and the only one used
    1: "BTN_SOUTH", # green (south)
    2: "BTN_WEST", # orange (north), seems switched
    3: "BTN_NORTH", # blue (west), seems switched
    4: "BTN_TR", # TR
    5: "BTN_TL", # TL
    6: "BTN_THUMBR", # pressing R throttle 
    7: "BTN_THUMBL", # pressing L throttle
    8: "BTN_START", # START
    9: "BTN_SELECT", # maybe logitic button, seems unsed anyway
}

axis_codes = {
    0: "ABS_X", # front wheel turning, contribution in left-right direction
    1: "ABS_Y", # front wheel turning, contribution in up-down direction, never called
    2: "ABS_RX", # rear wheel turning, contribution in left-right direction
    3: "ABS_RY", # rear wheel turning, contribution in up-down direction, never called
    4: "ABS_HAT0X", # left_arrow (-1), right_arrow (+1)
    5: "ABS_HAT0Y", # up_arrow (-1), down_arrow (+1)
    6: "ABS_Z", # LT (+1)
    7: "ABS_RZ", # RT (+1)
}

r_btc = {value: key for key, value in button_codes.items()}
r_atc = {value: key for key, value in axis_codes.items()}


#Wheel pin voltage control
kit.servo[0].set_pulse_width_range(0, 5000) # 19000) # front/rear motors
kit.servo[2].set_pulse_width_range(0, 5000) # 19000) # front/rear motors

#Servo pin voltage pwm control
kit.servo[1].set_pulse_width_range(1000,2000)
kit.servo[3].set_pulse_width_range(1000,2000)

kit.servo[1].actuation_range = 180
kit.servo[3].actuation_range = 180

#turning cam
#kit.servo[4].set_pulse_width_range(500,2500)
#kit.servo[5].set_pulse_width_range(500,2500)

#kit.servo[4].actuation_range = 180
#kit.servo[5].actuation_range = 180


print('Basic servo motion')
motor_pin_a.value = True  
motor_pin_b.value = True   

kit.servo[1].angle = 90 - 75

time.sleep(1)
kit.servo[0].angle = 180
kit.servo[2].angle = 0
time.sleep(3)
kit.servo[0].angle = 0 
kit.servo[2].angle = 0 
time.sleep(3)
kit.servo[0].angle = 0 
kit.servo[2].angle = 180
time.sleep(3)

# turn off servos
kit.servo[2].angle = 0
time.sleep(1)
# 90 seems to be neutral?
kit.servo[1].angle = 90
time.sleep(1)
print('Set servo angles')
