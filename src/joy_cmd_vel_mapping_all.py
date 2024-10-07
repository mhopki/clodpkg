#%%
class JoyListener:
    def __init__(self):

        self.last_received_time = rospy.Time.now()
        self.last_joy_message = None

        self.drive_req = 0.0
        self.braking = False
        self.turning = False
        self.turning2 = False
        self.t_out = 0.0
        self.t_out2 = 0.0
        self.m_out = 0.0
        self.m_in1 = 0
        self.m_in2 = 0
        self.m_inc = 20
        self.cy_out = 0
        self.cp_out = 0
        self.cam_yaw = False
        self.cam_pitch = False
        self.cy_pos = 90
        self.cp_pos = 0
        self.offset_wf = 10
        self.offset_wb = -5

    def joy_callback(self, data):
        rospy.loginfo('Callback received')
        # Update the last received time and message
        self.last_received_time = rospy.Time.now()
        self.last_joy_message = data

    def cmd_robot_stop(self, time_since_last_receive):
   
        # Set brake if no received msg recently
        if time_since_last_receive.to_sec() > 1.0:
        # 90-0, turns towards left, 0 is all the way to the left 
        # 180 is all the way to the right 
            self.drive_req = 0
            motor_pin_a.value = True 
            motor_pin_b.value = True 
            kit.servo[0].angle = 90
            kit.servo[2].angle = 90
            # kit.continuous_servo[0].throttle = 1
            
            # kit.continuous_servo[2].throttle = 1

            rospy.loginfo('No Joy message received. Robot has stopped. 90')
            rospy.loginfo(f'{time_since_last_receive.to_sec()}')


    def spin(self):
        rate = rospy.Rate(1000) # 10Hz
        while not rospy.is_shutdown():
            # Check if we have received a message in the last 2 seconds
            time_since_last_receive = rospy.Time.now() - self.last_received_time
            
            self.cmd_robot_stop(time_since_last_receive)
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('joy_listener', anonymous=True)
    joy_sub = rospy.Subscriber('/joy', Joy, self.joy_callback, queue_size=1, tcp_nodelay=True)
    joy_listener = JoyListener()
    rate = rospy.Rate(1000)
    joy_listener.spin()
