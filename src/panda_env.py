#!/usr/bin/env python

import os
import sys
import subprocess
import signal

import numpy as np

import rospy
import roslaunch
import moveit_commander
import panda_client as panda
from geometry_msgs.msg import Twist
from franka_msgs.msg import FrankaState
from franka_control.msg import ErrorRecoveryActionGoal


# PANDA_REAL='/home/florian/code/catkin_ws/src/panda_real/launch'
PANDA_REAL='/home/melissa/Workspace/RLProject/Panda/panda_real/launch'

class FrankaPandaRobotBase(object):

    def __init__(self):
        # Initialize ROS node.
        rospy.init_node("panda_arm_env", disable_signals=True)
        # Ctrl/C keyboard node exit handler.
        signal.signal(signal.SIGINT, lambda sig, frame: signal_handler(sig, frame, self) )
        
        # Ros sleep rate. This ensures there is enough delay in between
        # controller switch calls. If not set appropriately, it can cause
        # undesirable behaviour during controller switch.
        self.rate = rospy.Rate(1.0 / 5.0) # 0.2hz
        self.dt = 0.2
        self.step_size = rospy.Rate(1.0 / self.dt) # 5hz

        self.home_pose = None
        self.cur_pos = None
        self.last_pos = None
        self.action_space = self.ActionSpace()
        self.max_u = 2.0
        
        self.velocity_publisher = rospy.Publisher("/franka_control/target_velocity", Twist, queue_size=1)

        self.panda_arm_state_sub =  rospy.Subscriber(
            "/franka_state_controller/franka_states",
            FrankaState,
            self._state_callback)

        self.panda_arm_velocity =  rospy.Subscriber(
            "/franka_control/current_velocity",
            Twist,
            self._velocity_callback)
        
        # Safety zone constraint based on joint positions published on
        # /franka_state_controller/franka_states. Order is x,y,z.
        # Forward/Backward [0.33, 0.7]
        # Left/Right [-0.4, 0.35]
        # Up/Down [0.005, 0.32]
        self.safety_region = np.array([[0.4, 0.6], [-0.2, 0.2], [0.15, 0.4]])

        self.use_home_estimate = True

        self.error_recovery_pub = rospy.Publisher("/franka_control/error_recovery/goal",
            ErrorRecoveryActionGoal, queue_size=1)

        # Sleep to give the publishers above a chance to run.
        self.ros_sleep = 1.0
        rospy.sleep(self.ros_sleep)


        # use position control to recover
        self.enable_pos_control()
        moveit_commander.roscpp_initialize(sys.argv)
        #self.scene = moveit_commander.PlanningSceneInterface()
        self.panda_client = panda.PandaClient()
        self.panda_client.go_home()
        self.disable_pos_control()
        self.enable_vel_control()

        # Store the initial home pose
        self.home_pose = self.cur_pos
        print("Home Position is: ", self.home_pose)


    def run_go_to_boundary_test(self):
        actions = (
            [-1.0, -1.0, -1.0],
            [+1.0, -1.0, -1.0],
            [+1.0, +1.0, -1.0],
            [-1.0, +1.0, -1.0],
            [-1.0, -1.0, +1.0],
            [+1.0, -1.0, +1.0],
            [+1.0, +1.0, +1.0],
            [-1.0, +1.0, +1.0],
        )

        counter = 0
        self.reset()
        while not rospy.is_shutdown():
            for action in actions:
                for _ in range(50):
                    self.step(action)

    def run_controller_switch_test(self):
        action = [0.8, 0.0, 0.0]

        counter = 0
        while not rospy.is_shutdown():

            if counter % 100 == 0:
                action[0] = -action[0]
                
            self.apply_velocity_action(action)
            
            counter += 1
            rospy.sleep(0.033)

            if counter % 300 == 0 and counter >= 300:
                self.reset()
                action[0] = -action[0]

    class ActionSpace:
        def __init__(self, seed=0):
            self.random = np.random.RandomState(seed)
            self.shape = (3,)

        def sample(self):
            return self.random.rand(3)

    def compute_reward(self, achieved_goal, desired_goal, info=0):
        raise NotImplementedError

    def seed(self, seed):
        return
    
    def render(self):
        return
        
    def step(self, action):
        action = self._process_action(action)

        self.apply_velocity_action(action)
        self.step_size.sleep()

        return self._get_obs()

    def reset(self):
        
        self._move_up()

        if self.use_home_estimate:
            self._go_to_est_home()
            self._stop()
        else:
            self._stop()
            self.disable_vel_control()
            try:
                self.enable_pos_control()
                self.panda_client.go_home()
                self.disable_pos_control()
                
            except rospy.ROSInterruptException:
                print('Interrupted before completion during reset.')
                return

            self.enable_vel_control()
        
        self.last_pos = self.cur_pos
        return self._get_obs()[0]
    
    def _move_up(self):
        """If too close to the boundaries of the hole,
        move up slightly, before returning home.
        """
        if not self.cur_pos[2] < 0.15:
            return
        goal = (np.array(self.cur_pos) + np.array([0.0, 0.0, 0.1]))
        self._go_to_goal(goal)
    
    def _go_to_est_home(self):
        self._go_to_goal(self.home_pose)
    
    def _go_to_goal(self, goal):
        max_u = self.max_u
        self.max_u = 4.0
        last_pos = self.cur_pos
        while ((np.linalg.norm(goal - self.cur_pos) >= 0.01) or (np.linalg.norm(last_pos - self.cur_pos) >= 0.01)) :
            action = (goal - self.cur_pos) * 10.0
            action = self._process_action(action)
            last_pos = self.cur_pos
            self.apply_velocity_action(action)
            self.step_size.sleep()
        self.max_u = max_u        

    def send_control_recovery_message(self):
        """Publish an empty `ErrorRecoveryActionGoal`
        to the topic `/franka_control/error_recovery/goal`.
        This ensures all previous exceptions occured controller
        are cleared.
        """
        empty_recovery_msg = ErrorRecoveryActionGoal()
        self.error_recovery_pub.publish(empty_recovery_msg)

    def enable_vel_control(self):
        try:
            uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
            roslaunch.configure_logging(uuid)
            self.velocity_control_launcher = roslaunch.parent.ROSLaunchParent(
                uuid, [os.path.join(PANDA_REAL, "franka_arm_vel_controller.launch")])
            
            self.velocity_control_launcher.start()
            self.rate.sleep()
            rospy.loginfo("STARTED VELOCITY CONTROL")

        except rospy.ROSInterruptException:
            print('Enable velocity launch failed.')
            return

    def disable_vel_control(self):
        self.velocity_control_launcher.shutdown()
        self.rate.sleep()
        rospy.loginfo("SHUT DOWN VELOCITY CONTROL")


    def enable_pos_control(self):
        try:
            uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
            roslaunch.configure_logging(uuid)
            
            self.position_control_launcher = roslaunch.parent.ROSLaunchParent(
                uuid, [os.path.join(PANDA_REAL, "panda_moveit.launch")])
            
            self.position_control_launcher.start()
            self.rate.sleep()
            rospy.loginfo("STARTED POSITION CONTROL")
            
        except rospy.ROSInterruptException:
            print('Enable position launch file failed.')
            return

    def disable_pos_control(self):
        self.position_control_launcher.shutdown()
        self.rate.sleep()
        rospy.loginfo("SHUT DOWN POSITION CONTROL")
        
    def enable_grip(self):
        pass

    def disable_grip(self):
        pass

    def apply_velocity_action(self, action):

        twist = Twist()
        twist.linear.x = action[0]
        twist.linear.y = action[1]
        twist.linear.z = action[2]
        # Call as appropriate - Ideally this should be
        # called when we have determined the controller
        # is stuck due to robot being in an invalid pose
        # or due to collisions that can be recovered by
        # the agent.
        self.send_control_recovery_message()
        self.velocity_publisher.publish(twist)

    def _process_action(self, action):
        cur_pos = self.cur_pos
        clip_min = np.where(cur_pos > self.safety_region[:, 0], -self.max_u, 0.0)
        clip_max = np.where(cur_pos < self.safety_region[:, 1], self.max_u, 0.0)
        action = action[:3]
        action = np.clip(action, clip_min, clip_max)
        return action
        
    def _get_obs(self):
        raise NotImplementedError

    def _stop(self):
        self.apply_velocity_action((0,0,0))
        rospy.sleep(self.ros_sleep)

    def _state_callback(self, data):
        self.cur_pos = np.asarray(data.O_T_EE[12:15])
    
    def _velocity_callback(self, data):
        pass

class FrankaPegInHole(FrankaPandaRobotBase):
    
    def __init__(self):
        super(FrankaPegInHole, self).__init__()
        self.goal = np.array((0.455, -0.005, 0.15))
        self.threshold = 0.05
        self.sparse = False
        self._max_episode_steps = 50
    
    def compute_reward(self, achieved_goal, desired_goal, info=0):
        distance = self._compute_distance(achieved_goal, desired_goal)
        if self.sparse == False:
            return -distance
            # return np.maximum(-0.5, -distance)
            # return 0.05 / (0.05 + distance)
        else:  # self.sparse == True
            return -(distance >= self.threshold).astype(np.int64)

    def _compute_distance(self, achieved_goal, desired_goal):
        achieved_goal = achieved_goal.reshape(-1, 3)
        desired_goal = desired_goal.reshape(-1, 3)
        distance = np.sqrt(np.sum(np.square(achieved_goal - desired_goal), axis=1))
        return distance

    def _get_obs(self):
        """
        Get observation
        """
        pos = self.cur_pos
        vel = (pos - self.last_pos) / self.dt
        self.last_pos = pos
        obs = np.concatenate((pos, vel), axis=0)
        ag = obs[:3].copy()
        r = self.compute_reward(ag, self.goal)
        # return distance as metric to measure performance
        distance = self._compute_distance(ag, self.goal)
        # is_success or not
        is_success = distance < self.threshold
        return (
            {"observation": obs, "desired_goal": self.goal, "achieved_goal": ag},
            r,
            0,
            {"is_success": is_success, "shaping_reward": -distance},
        )   


def signal_handler(sig, frame, panda_robot):
    print ("FINISHED EXPERIMENT")
    panda_robot._stop()
    sys.exit(0)
    
if __name__ == '__main__':
    panda_robot = FrankaPegInHole()
    #panda_robot.run_controller_switch_test()
    panda_robot.run_go_to_boundary_test()
        

    
