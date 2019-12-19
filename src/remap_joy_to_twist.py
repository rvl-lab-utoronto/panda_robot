#!/usr/bin/python

import numpy as np

import rospy
import subprocess

from sensor_msgs.msg import Joy
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from actionlib_msgs.msg import GoalID

from franka_msgs.msg import FrankaState


class JoyToTwist:
    def __init__(self):
        # Map buttons to re-adjust the speed.
        # This would be maximum.
        self.speed_setting = 1
        self.cmd_vel_pub = rospy.Publisher(
            "/franka_control/target_velocity", Twist, queue_size=1)
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
        twist = Twist()

        # X - Forward/Backward
        # Y - Left/Right
        # Z - Up/Down

        if self.enable_safe_zone:

            ############
            # Linear.x
            ############
            if self.franka_current_state[0] >= self.safety_region[0][0] and \
                    self.franka_current_state[0] <= self.safety_region[0][1]:
                twist.linear.x = self.vel_scale * data.axes[1]

            ############
            # Linear.y
            ############
            if self.franka_current_state[1] >= self.safety_region[1][0] and \
                    self.franka_current_state[1] <= self.safety_region[1][1]:
                #print('Controlling y')
                twist.linear.y = self.vel_scale * data.axes[0]

            ############
            # Linear.z
            ############
            if self.franka_current_state[2] >= self.safety_region[2][0] and \
                    self.franka_current_state[2] <= self.safety_region[2][1]:
                #print('Controlling z')
                twist.linear.z = self.vel_scale * data.axes[4]

            if self.allow_pose_correction:
                ####################################
                # Linear.x allow forward correction
                ####################################
                if self.franka_current_state[0] < self.safety_region[0][0]:
                    # In this mode, we only accept forward/positive command
                    if data.axes[1] > 0:
                        twist.linear.x = self.vel_scale * data.axes[1]

                ####################################
                # Linear.x allow backward correction
                ####################################
                if self.franka_current_state[0] > self.safety_region[0][1]:
                    # In this mode, we only accept backward/negative command
                    if data.axes[1] < 0:
                        twist.linear.x = self.vel_scale * data.axes[1]

                ####################################
                # Linear.y allow right correction
                ####################################
                if self.franka_current_state[1] < self.safety_region[1][0]:
                    # In this mode, we only accept right/negative command
                    if data.axes[0] > 0:
                        twist.linear.y = self.vel_scale * data.axes[0]

                ####################################
                # Linear.y allow left correction
                ####################################
                if self.franka_current_state[1] > self.safety_region[1][1]:
                    # In this mode, we only accept left/positive command
                    if data.axes[0] < 0:
                        twist.linear.y = self.vel_scale * data.axes[0]

                ####################################
                # Linear.z allow down correction
                ####################################

                if self.franka_current_state[2] < self.safety_region[2][0]:
                    # In this mode, we only accept forward command
                    if data.axes[4] > 0:
                        twist.linear.z = self.vel_scale * data.axes[4]

                ####################################
                # Linear.z allow up correction
                ####################################

                if self.franka_current_state[2] > self.safety_region[2][1]:
                    # In this mode, we only accept backward command
                    if data.axes[4] < 0:
                        twist.linear.z = self.vel_scale * data.axes[4]

        else:

            twist.linear.x = self.vel_scale * data.axes[1]
            twist.linear.y = self.vel_scale * data.axes[0]
            twist.linear.z = self.vel_scale * data.axes[4]

        # Note: twist.angular are ignored, so we don't set them.
        self.cmd_vel_pub.publish(twist)

    def on_franka_state(self, data):
        # x-pose
        self.franka_current_state[0] = data.O_T_EE[12]
        # y-pose
        self.franka_current_state[1] = data.O_T_EE[13]
        # z-pose
        self.franka_current_state[2] = data.O_T_EE[14]

        print(self.franka_current_state)


def main():
    rospy.init_node("joy_to_twist")
    controller = JoyToTwist()
    while not rospy.is_shutdown():
        rospy.sleep(0.1)


if __name__ == '__main__':
    main()
