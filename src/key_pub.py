#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from pynput import keyboard

def on_key_event(key, is_press):
    try:
        key_str = key.char  # Get character key
    except AttributeError:
        key_str = str(key)  # Use other key representation

    event_type = "pressed" if is_press else "released"
    rospy.loginfo(f"Key {key_str} {event_type}")

    key_publisher.publish(key_str)

def on_key_press(key):
    on_key_event(key, True)

def on_key_release(key):
    on_key_event(key, False)

rospy.init_node('keyboard_publisher', anonymous=True)
key_publisher = rospy.Publisher('/keypress', String, queue_size=10)

with keyboard.Listener(on_press=on_key_press, on_release=on_key_release) as listener:
    rospy.spin()
