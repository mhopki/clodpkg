#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Joy

import time
from adafruit_servokit import ServoKit

#import Jetson.GPIO as GPIO
import board
import digitalio

#from visual

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

kit.servo[0].set_pulse_width_range(500,10000)

class JoyListener:
    def __init__(self):
        rospy.init_node('joy_listener', anonymous=True)
        self.joy_sub = rospy.Subscriber('/joy', Joy, self.joy_callback)
        self.last_received_time = rospy.Time.now()
        self.last_joy_message = None
        self.drive_req = 0.0
        self.braking = False
        self.turning = False
        self.t_out = 0.0
        self.m_out = 0.0

        # ThrustHeading subscriber form visual serrvoing node
        #self.joy_sub = rospy.Subscriber('/joy', Joy, self.thrustheading_callback)

    def joy_callback(self, data):
        # Update the last received time and message
        self.last_received_time = rospy.Time.now()
        self.last_joy_message = data

    def spin(self):
        rate = rospy.Rate(10) # 10Hz
        while not rospy.is_shutdown():
            # Check if we have received a message in the last 2 seconds
            time_since_last_receive = rospy.Time.now() - self.last_received_time
            #if time_since_last_receive.to_sec() < 2.0:
                # Print the last received message
            if self.last_joy_message != None:
                #Turn Command
                if abs(self.last_joy_message.axes[r_atc["ABS_X"]]) > 0:
                    self.turning = True
                    self.t_out = self.last_joy_message.axes[r_atc["ABS_X"]]
                    #print("Turning")
                elif self.turning == True:
                    self.turning = False
                #Forward Command
                if self.last_joy_message.axes[r_atc["ABS_RZ"]] > 0:#7
                    if (self.drive_req == 0 and self.braking == False):
                        print("Start Forward")
                        #print(self.drive_req)
                        self.drive_req = 0.5
                    elif (self.drive_req >= 0.5 and self.drive_req < 1.0):
                        self.drive_req += 0.1
                        #print(self.drive_req)
                    elif (self.drive_req >= 1.0):
                        self.m_out = self.last_joy_message.axes[r_atc["ABS_RZ"]]
                elif self.drive_req > 0:
                    print("Stop Forward")
                    self.drive_req = 0
                #Reverse Command
                if self.last_joy_message.axes[r_atc["ABS_Z"]] > 0:#6
                    if (self.drive_req == 0 and self.braking == False):
                        print("Start Backward")
                        #print(self.drive_req)
                        self.drive_req = -0.5
                    elif (self.drive_req <= -0.5 and self.drive_req > -1.0):
                        self.drive_req -= 0.1
                        #print(self.drive_req)
                    elif (self.drive_req <= -1.0):
                        self.m_out = self.last_joy_message.axes[r_atc["ABS_Z"]]
                elif self.drive_req < 0:
                    print("Stop Backward")
                    self.drive_req = 0
                #Brake Command
                if self.last_joy_message.buttons[r_btc["BTN_EAST"]] == 1:
                    self.braking = True
                elif self.braking == True:
                    self.braking = False
            
            if self.turning == True:
                servo_val = self.t_out
                if servo_val < 0:
                    servo_val /= 1.5 / 1.5
                if servo_val > 0:
                    servo_val /= 1.5 / 1.5
                servo_val = (servo_val + 1) * 45 + 39 #59
                print(servo_val, self.t_out)
                kit.servo[1].angle = servo_val
            else:
                servo_val = 0
                servo_val = (servo_val + 1) * 45 + 39 #59
                #print(servo_val, self.t_out)
                kit.servo[1].angle = servo_val

            #Forward Driving
            if self.drive_req >= 0.5 and self.drive_req < 0.75:
                #Brake first
                poop = 0
                #motor_pin_a.value = False
                #motor_pin_b.value = False
            elif self.drive_req >= 0.75 and self.drive_req < 1.0:
                #Idle mid
                motor_pin_a.value = True
                motor_pin_b.value = True
            elif self.drive_req >= 1.0:
                #Now Drive
                motor_pin_a.value = True
                motor_pin_b.value = False

                motor_val = self.m_out
                motor_val = (motor_val) * 180
                #print("m_val: ", motor_val, self.m_out)
                kit.servo[0].angle = motor_val
                print("Driving Forward")
            #Backward Driving
            if self.drive_req <= -0.5 and self.drive_req > -0.75:
                #Brake first
                poop = 0
                #motor_pin_a.value = False
                #motor_pin_b.value = False
            elif self.drive_req <= -0.75 and self.drive_req > -1.0:
                #Idle mid
                motor_pin_a.value = True
                motor_pin_b.value = True
            elif self.drive_req <= -1.0:
                motor_pin_a.value = False
                motor_pin_b.value = True

                motor_val = self.m_out
                motor_val = (motor_val) * 180
                #print("m_val:", motor_val, self.m_out)
                kit.servo[0].angle = motor_val
                print("Driving Backward")
            #Braking
            if self.braking == True:
                self.drive_req = 0
                motor_pin_a.value = False
                motor_pin_b.value = False
                print("Braking")
            #Idling
            if self.drive_req == 0 and self.braking == False:
                motor_pin_a.value = True
                motor_pin_b.value = True
            #rospy.loginfo("Received joy message: %s", str(self.last_joy_message))
            rate.sleep()


if __name__ == '__main__':
    joy_listener = JoyListener()
    joy_listener.spin()
