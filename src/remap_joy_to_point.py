#!/usr/bin/python

import numpy as np

import rospy
import subprocess

from sensor_msgs.msg import Joy
from std_msgs.msg import Float32
from geometry_msgs.msg import Point
from actionlib_msgs.msg import GoalID

from franka_msgs.msg import FrankaState


class JoyToPose:
    def __init__(self):
        # Map buttons to re-adjust the speed.
        # This would be maximum.
        self.speed_setting = 1
        self.cmd_pos_pub = rospy.Publisher(
            "/franka_control/target_dpoint", Point, queue_size=1)
        self.joy_sub = rospy.Subscriber("/joy", Joy, self.on_joy)
        self.state_sub = rospy.Subscriber(
            "/franka_state_controller/franka_states",
            FrankaState,
            self.on_franka_state)

        self.vel_scale = 2.0
        self.franka_current_state = np.array([0.0, 0.0, 0.0])

        # Safety zone constraint based on joint positions published on
        # /franka_state_controller/franka_states. Order is x,y,z.
        # Up/Down [0.005, 0.32]
        # Forward/Backward [0.33, 0.7]
        # Left/Right [-0.4, 0.35]
        self.safety_region = np.array(
            [[0.33, 0.7], [-0.4, 0.35], [0.005, 0.41]])

        self.enable_safe_zone = True
        self.allow_pose_correction = True

    def on_joy(self, data):
        # Publish Twist
        pose = Point()

        # X - Forward/Backward
        # Y - Left/Right
        # Z - Up/Down
        pose.x = data.axes[1]
        pose.y = data.axes[0]
        pose.z = data.axes[4]

        # Note: pose.y is ignored?
        self.cmd_pos_pub.publish(pose)

    def on_franka_state(self, data):
        # x-pose
        self.franka_current_state[0] = data.O_T_EE[12]
        # y-pose
        self.franka_current_state[1] = data.O_T_EE[13]
        # z-pose
        self.franka_current_state[2] = data.O_T_EE[14]

        #print(self.franka_current_state)


def main():
    rospy.init_node("joy_to_pose")
    controller = JoyToPose()
    while not rospy.is_shutdown():
        rospy.sleep(0.1)


if __name__ == '__main__':
    main()
