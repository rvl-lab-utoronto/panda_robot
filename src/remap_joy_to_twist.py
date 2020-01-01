#!/usr/bin/python

import numpy as np
import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

from panda_client import PandaClient

class JoyToTwist:
    def __init__(self):
        # Map buttons to re-adjust the speed.
        # This would be maximum.
        self.vel_scale = 1.0
        
        # Safety zone constraint based on joint positions published on
        # /franka_state_controller/franka_states. Order is x,y,z.
        # Up/Down [0.005, 0.32]
        # Forward/Backward [0.33, 0.6]
        # Left/Right [-0.4, 0.35]
        self.safety_region = np.array([[0.33, 0.7], [-0.3, 0.05], [0.05, 0.41]])
        self.panda_client = PandaClient(ee_safety_zone=self.safety_region) 
        
        self.time_since_last_collision = rospy.Time.now()
        self.last_time_home_button_pressed = rospy.Time.now()
        
        self.joy_sub = rospy.Subscriber("/joy", Joy, self.joy_callback)
        
    def joy_callback(self, data):
        # Publish Twist
        twist = Twist()

        # X - Forward/Backward
        # Y - Left/Right
        # Z - Up/Down

        twist.linear.x = self.vel_scale * data.axes[1]
        twist.linear.y = self.vel_scale * data.axes[0]
        twist.linear.z = self.vel_scale * data.axes[3]
                
        # Pressed red button, go to home config
        if data.buttons[2] > 0 and (rospy.Time.now() - self.last_time_home_button_pressed).to_sec() > 5:
            self.last_time_home_button_pressed = rospy.Time.now()
            self.panda_client.go_home()
            return
        
        current_time = rospy.Time.now()
        if self.panda_client.is_in_collision_mode():
            self.time_since_last_collision = current_time
            

        if self.panda_client.is_in_move_mode() and (current_time - self.time_since_last_collision).to_sec() > 2.0:
            # Note: twist.angular are ignored, so we don't set them.
            self.panda_client.set_ee_velocity(twist)
        else:
            self.panda_client.set_ee_velocity(self.panda_client.current_target_velocity)
           
        
        
    
def main():
    rospy.init_node("joy_to_twist")
    env = JoyToTwist()
    env.panda_client.set_collision_behavior()
    
    while not rospy.is_shutdown():
        if env.panda_client.is_in_collision_mode():
            print ('Error')
            res = env.panda_client.recover_from_errors()
            print (res)
            
        rospy.sleep(0.1)


if __name__ == '__main__':
    main()
