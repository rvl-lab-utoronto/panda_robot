#!/usr/bin/env python

import sys
import signal

import numpy as np

import rospy
from panda_client import PandaClient
from geometry_msgs.msg import Twist

import gym
from gym import error, spaces, utils
from gym.utils import seeding

#
# Note: every time you change the environment
# do not forget to pip install it system-wide
# by going to path/to/panda_robot/src/gym-panda-robot and doing
# sudo pip install --upgrade --force-reinstall -e .
#

class PandaReacherEnv(gym.Env):
    metadata = {'render.modes': ['human']}
    
    def __init__(self):
        super(PandaReacherEnv, self).__init__()
        rospy.init_node('panda_robot_reacher_v0', anonymous=True)

        dt = 0.1
        self.rate = rospy.Rate(1.0/dt) # apply policy at 10Hz
        self.panda_client = PandaClient(ee_safety_zone=[[0.2, 0.7], [-0.3, 0.05], [0.15, 0.4]])
        self.hole_goal = np.array((0.455, -0.005, 0.25))
        self.threshold = 0.05
        self.sparse = False
        self._max_episode_steps = 50

    def apply_velocity_action(self, action):
        twist = Twist()
        twist.linear.x = action[0]
        twist.linear.y = action[1]
        twist.linear.z = action[2]
        self.panda_client.set_ee_velocity(twist)
    
    def compute_reward(self, achieved_goal, desired_goal, info=0):
        distance = self.compute_distance(achieved_goal, desired_goal)
        if not self.sparse:
            return -distance
            # return np.maximum(-0.5, -distance)
            # return 0.05 / (0.05 + distance)
        else:  # self.sparse == True
            return -(distance >= self.threshold).astype(np.int64)

    def compute_distance(self, achieved_goal, desired_goal):
        achieved_goal = achieved_goal.reshape(-1, 3)
        desired_goal = desired_goal.reshape(-1, 3)
        distance = np.sqrt(np.sum(np.square(achieved_goal - desired_goal), axis=1))
        return distance

    def preprocess_action(self, action):
        return action
    
    def step(self, action):
        action = self.preprocess_action(action)
        self.apply_velocity_action(action)
        self.rate.sleep()

        obs = self.get_obs()
        achieved_goal = obs[:3].copy()
        reward = self.compute_reward(achieved_goal, self.hole_goal)

        distance = self.compute_distance(achieved_goal, self.hole_goal)
        done = distance < self.threshold
        info = {'shaped_reward': -distance}
        
        return (obs, reward, done, info)
    
    def reset(self):
        self.panda_client.go_home()
        obs = self.get_obs()
        return obs
        
    def get_obs(self):
        pos = self.panda_client.O_P_EE
        vel = self.panda_client.O_V_EE
        obs = np.concatenate((pos, vel), axis=0)
        return obs

    def close(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        self.panda_client.set_ee_velocity(twist)
        
    
if __name__ == '__main__':
    panda_robot = PandaReacherEnv()
    panda_robot.run_go_to_boundary_test()
        

    
