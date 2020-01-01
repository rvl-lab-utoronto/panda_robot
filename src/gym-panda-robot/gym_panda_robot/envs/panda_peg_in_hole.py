#!/usr/bin/env python

import sys
import signal
import numpy as np

import rospy
import roslaunch
import moveit_commander
import panda_client as panda
from geometry_msgs.msg import Twist
from panda_base_env import PandaBaseEnv

class PandaPegInHoleEnv(PandaBaseEnv):
    
    def __init__(self):
        super(PandaPegInHoleEnv, self).__init__()
        self.hole_goal = np.array((0.455, -0.005, 0.15))
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
        distance = self._compute_distance(achieved_goal, desired_goal)
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

    def step(self):
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
        
    def get_obs(self):
        pos = self.panda_client.O_P_EE
        vel = self.panda_client.O_V_EE
        obs = np.concatenate((pos, vel), axis=0)
        return obs
    

def signal_handler(sig, frame, panda_robot):
    print ("FINISHED EXPERIMENT")
    panda_robot._stop()
    sys.exit(0)
    
if __name__ == '__main__':
    panda_robot = PandaPegInHoleEnv()
    panda_robot.run_go_to_boundary_test()
        

    
