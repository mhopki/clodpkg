#!/usr/bin/env python3

# TODO: update CMakesList and package to include custom message. 
import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32MultiArray


import time
from adafruit_servokit import ServoKit

#import Jetson.GPIO as GPIO
import board
import digitalio

# Set channels to the number of servo channels on your kit.
# 8 for FeatherWing, 16 for Shield/HAT/Bonnet.
# kit = ServoKit(channels=16)

# Pin Definitions
# motor_pin_a = digitalio.DigitalInOut(board.D27)#13  # BOARD pin 12, BCM pin 18
# motor_pin_a.direction = digitalio.Direction.OUTPUT
# motor_pin_b = digitalio.DigitalInOut(board.D18)#12  # BOARD pin 16, BCM pin 16
# motor_pin_b.direction = digitalio.Direction.OUTPUT


# there are 12 indices in the /joy/button, but only 10 here, and only '0' is used.
# we shouldn't need a breaking button if the controller is mapped correctly
# letting go of throttle should be equivalent to stopping
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





# Sets 4 wheel drive when both are True, otherwise 2 wheel drive
# motor_pin_a.value = True 
# motor_pin_b.value = True 

class TwistToMotors:
    def __init__(self):
        rospy.init_node('twist_to_motors', anonymous=True)
        self.joy_subscriber = rospy.Subscriber('/joy', Joy, self.joy_callback, queue_size=1, tcp_nodelay=True)
        self.motor_cmd_publisher = rospy.Publisher('/motor_cmd', Float32MultiArray, queue_size=100)
        
        # self.initialize_servos()
        self.last_received_time = rospy.Time.now()
        
        # Initializing last joy message
        self.last_joy_message = Joy()
        self.last_joy_message.axes = [0]*8
        self.last_joy_message.axes = [0]*12
        
    def initialize_servos(self):
        rospy.loginfo('Initializing servos.')
        
        # Motor min and maximum throttle.
        # This max is fast if set at angle=180 = full throttle
        kit.servo[0].set_pulse_width_range(0,19000) # front/rear motors
        kit.servo[2].set_pulse_width_range(0,19000) # front/rear motors

        #Servo pin voltage pwm control
        kit.servo[1].set_pulse_width_range(1000,2000)
        kit.servo[3].set_pulse_width_range(1000,2000)

        # TODO: check this limit for safety and physical constraints.
        kit.servo[1].actuation_range = 30 # 180
        kit.servo[3].actuation_range = 30 # 180


    def joy_callback(self, data):
        rospy.loginfo('Callback received')
        
        current_time = rospy.Time.now()
        
        # Update the last received time and message
        self.last_received_time = rospy.Time.now()
        self.last_joy_message = data
            
    def cmd_motors_reverse(self, throttle_input):
        rospy.loginfo(f'Reversing with {throttle_input * 100} % Throttle')
         
        # The max value of the servos are 180, and button_held_down ranges from [0,1]
        # kit.servo[2].angle = 180 * throttle_input
        # kit.servo[0].angle = 0 
        # angles = (kit.servo[0].angle, kit.servo[2].angle)
        angles = (180 * throttle_input, 0)
        
        # Unsure whether this is the best way, but emperically seems relatively smooth
        # Ideally, we don't require a period of sleep, but it seems the motors will not move
        # Without it?
        sleep_duration = max(0.05, throttle_input)
        time.sleep(sleep_duration)
        # kit.servo[2].angle = 0

        return angles

    def cmd_motors_forward(self, throttle_input):
        # TODO: maybe don't need two separate functions, since only the pin index is different.
        rospy.loginfo(f'Forward Motion {throttle_input * 100} % Throttle')

        # The max value of the servos are 180, and button_held_down ranges from [0,1]
        scaled_throttle = 180 * throttle_input
        # kit.servo[0].angle = scaled_throttle
        # kit.servo[2].angle = 0
        # angles = (kit.servo[0].angle, kit.servo[2].angle)
        angles = (scaled_throttle, 0) 
        
        # Unsure whether this is the best way, but emperically seems relatively smooth
        # Ideally, we don't require a period of sleep, but it seems the motors will not move
        # Without it?
       
        sleep_duration = max(0.05, throttle_input)
        time.sleep(sleep_duration)
        
        # Also, resetting the pulse width seems to make the most sense
        # kit.servo[0].angle = 0
        
        return angles
    
    def cmd_servos_turn(self, steering_input):
        # TODO: do something with the angles, more turning vs less turning?
        # 0 = left, 90 = neutral, 180 = right
        if steering_input < 0:
            rospy.loginfo('Turning left.')
        elif steering_input > 0:
            rospy.loginfo('Turning right.')
        else: # value = 0 
            rospy.loginfo('No turning.')
        
        steering_input_limits = (-1, 1)
        servo_limits = (0, 180)
        scaled_steering_input = self.scale_values(steering_input, 
                                             steering_input_limits[0], steering_input_limits[1], 
                                             servo_limits[0], servo_limits[1])
                                                                                
        # For now assuming only the front wheels turn                                                                   
        # kit.servo[1].angle = scaled_steering_input
        # kit.servo[3].angle = 0
        # angles = (kit.servo[1].angle, kit.servo[3].angle)
        angles = (scaled_steering_input, 0)
        time.sleep(1)
        
        return angles 
                

    def cmd_move(self):
        print(self.last_joy_message)
        
        # Can understand why this joystick mapping is not ideal, as it allows for 
        # Forward and backward keys to be pressed at the same time
        # TODO: could update this to the official joy package
        
        # Forwards/Backwards. Must use if-else pairs for fowards/backwards motion because of gamepad mapping
        if self.last_joy_message.axes[r_atc['ABS_RZ']] > 0:
            motor_angles = self.cmd_motors_forward(self.last_joy_message.axes[r_atc['ABS_RZ']])
        elif self.last_joy_message.axes[r_atc['ABS_Z']] > 0:
            motor_angles = self.cmd_motors_reverse(self.last_joy_message.axes[r_atc['ABS_Z']])
        else: # possible to have no input from the controller
            motor_angles = (0, 0)
                
        # For steering
        # TODO check if its minus for left turns.
        servo_angles = self.cmd_servos_turn(self.last_joy_message.axes[r_atc['ABS_X']])
        
        motor_cmd_msg = self.generate_motor_cmd_msg(motor_angles, servo_angles)
        
        return motor_cmd_msg

    def generate_motor_cmd_msg(self, motor_angles, servo_angles):
        motor_pin0_angle = motor_angles[0]
        motor_pin2_angle = motor_angles[1]
        servo_pin1_angle = servo_angles[0]
        servo_pin3_angle = servo_angles[1]
        
        msg = Float32MultiArray()
        msg.data = [motor_pin0_angle, motor_pin2_angle, servo_pin1_angle, servo_pin3_angle]
    
        return msg 
                                
 
    def scale_values(self, value, source_min, source_max, target_min, target_max):
        # Scales a value from the range [source_min, source_max] 
        # to the range [target_min, target_max].
        return (value - source_min) * (target_max - target_min) / (source_max - source_min) + target_min
 
    def run(self):
        rate = rospy.Rate(100) 
        while not rospy.is_shutdown():
        
            # TODO: Check if this is even necessary. I think it is not, and actually might just get in the way.
            # If the time between two callback messages is greater than a second
            current_time = rospy.Time.now()
            time_difference = (current_time - self.last_received_time).to_sec()
            if time_difference > 1.0:
                rospy.loginfo('No Joy message received. Robot has stopped. 90')
                rospy.loginfo(f'Time since last message: {time_difference}')
                # kit.servo[0].angle = 0 
                # kit.servo[2].angle = 0
        
            # Move the robot    
            motor_cmd_msg = self.cmd_move()
            
            # Publish the current commanded PWM (throttle for servos, angle for steering)
            self.motor_cmd_publisher.publish(motor_cmd_msg)
        rate.sleep()
            
def main():        
    try:
        # Start with the robot stationary
        # kit.servo[0].angle = 0
        # kit.servo[2].angle = 0 
        
        # Initialize Subscriber 
        twist_to_motors_node = TwistToMotors()
        twist_to_motors_node.run()
        
    except rospy.ROSInterruptException:
    
        # End with the robot stationary 
        # kit.servo[0].angle = 0
        # kit.servo[2].angle = 0
        pass 
        
        
if __name__ == '__main__':
    rospy.loginfo('Subscribing to /joy, and manually controlling')
    main()

