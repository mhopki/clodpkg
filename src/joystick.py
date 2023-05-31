from inputs import devices, get_gamepad

import time
from adafruit_servokit import ServoKit

#import Jetson.GPIO as GPIO
import board
import digitalio
from sensor_msgs.msg import Joy

# Set channels to the number of servo channels on your kit.
# 8 for FeatherWing, 16 for Shield/HAT/Bonnet.
kit = ServoKit(channels=16)

# Pin Definitions
led = digitalio.DigitalInOut(board.D18)
led.direction = digitalio.Direction.OUTPUT
motor_pin_a = digitalio.DigitalInOut(board.D27)#13  # BOARD pin 12, BCM pin 18
motor_pin_a.direction = digitalio.Direction.OUTPUT
motor_pin_b = digitalio.DigitalInOut(board.D18)#12  # BOARD pin 16, BCM pin 16
motor_pin_b.direction = digitalio.Direction.OUTPUT

motor_pin_a.value = False
motor_pin_b.value = False

# Set both pins LOW to keep the motor idle
# You can keep one of them HIGH and the LOW to start with rotation in one direction 

global motor_power
motor_power = 0

global motor_power_r
motor_power_r = 0

global brake
brake = 0

"""
elif event.type == JOYAXISMOTION:
                    self.joy[event.joy].axis[event.axis] = event.value
                    #print(event.joy, " ", event.axis, " ", event.value)
                    servo_out = (self.joy[0].axis[0])
                    if servo_out < -1:
                        servo_out = -1.0
                    alt = servo_out
                    if servo_out < 0:
                        servo_out /= 2
                    servo_out = (servo_out + 1) * 45 + 90
            #(servo_out + 1) * 45 + 90 
                    print(servo_out)
                    kit.servo[1].angle = servo_out

                    servo_out = (self.joy[0].axis[5])
                    if servo_out < -1:
                        servo_out = -1.0
                    servo_out = (servo_out + 1) * 90
                    motor_power = servo_out
                    print(servo_out)
                    

                    servo_out = (self.joy[0].axis[2])
                    if servo_out < -1:
                        servo_out = -1.0
                    servo_out = (servo_out + 1) * 90
                    motor_power_r = servo_out
                    print(servo_out)
                elif event.type == JOYBALLMOTION:
                    self.joy[event.joy].ball[event.ball] = event.rel
                elif event.type == JOYHATMOTION:
                    self.joy[event.joy].hat[event.hat] = event.value
                elif event.type == JOYBUTTONUP:
                    self.joy[event.joy].button[event.button] = 0
                    brake = self.joy[0].button[1]
                elif event.type == JOYBUTTONDOWN:
                    self.joy[event.joy].button[event.button] = 1
                    brake = self.joy[0].button[1]
"""

# Loop indefinitely, reading input from the gamepad
#CHANGE TO BROADCAST TO ROSTOPIC!!!
while True:
    #print("mf")
    # Get the next event from the gamepad
    events = get_gamepad()
    # If the event is a button press, print the button code
    for event in events:
        
        if event.ev_type == "Absolute": #joystick and triggers
            #32767 is max for everything
            #Converstion to 255 for joystick is x/(32768)*255
            #IMPORTANT: count anything less than 1 as 0
            joylist = ["ABS_X","ABS_Y","ABS_RX","ABS_RY"]
            if (event.code in joylist):
                state_conv = event.state / (32768) * 255
                if abs(state_conv) < 5: state_conv = 0
                print(f"Axis {event.code}: {state_conv}")

                #turning
                if (event.code == "ABS_X"):
                    #print("turning")
                    servo_out = (state_conv)/255
                    if servo_out < -1:
                        servo_out = -1.0
                    if servo_out < 0:
                        servo_out /= 2
                    servo_out = (servo_out + 1) * 45 + 90
                    print(servo_out)
                    kit.servo[1].angle = servo_out
            else:
                #general
                x = 0
                #print(f"Axis {event.code}: {event.state}")
        elif event.ev_type == "Key": #Buttons
            #print(event.code, event.state)
            if (event.code == "BTN_EAST"):
                brake = event.state
                if (brake == 1): 
                    #lift brake button down
                    x = 0
                    #print("brake_down")
                else:
                    #lift brake button up
                    x = 0
                    #print("brake up")