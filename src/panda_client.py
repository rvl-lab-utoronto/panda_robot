#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
import actionlib
import franka_gripper
import franka_gripper.msg
import franka_msgs.msg
import numpy as np


def all_close(goal, actual, tolerance):
    """
    Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
    @param: goal       A list of floats, a Pose or a PoseStamped
    @param: actual     A list of floats, a Pose or a PoseStamped
    @param: tolerance  A float
    @returns: bool
    """
    all_equal = True
    if isinstance(goal, list):
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif isinstance(goal, geometry_msgs.msg.PoseStamped):
        return all_close(goal.pose, actual.pose, tolerance)

    elif isinstance(goal, geometry_msgs.msg.Pose):
        return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

    return True


class PandaClient(object):
    def __init__(self):
        super(PandaClient, self).__init__()

        # This is the outer-level interface to the robot:
        robot = moveit_commander.RobotCommander()

        # Instantiate a `MoveGroupCommander`_ object.  This object is an interface
        # to one group of joints.  In this case the group is the joints in the Panda
        # arm so we set ``group_name = panda_arm``.
        # This interface can be used to plan and execute motions on the Panda:
        group_name = "panda_arm"
        panda_arm = moveit_commander.MoveGroupCommander(group_name)
        panda_arm.set_end_effector_link('panda_EE')
        panda_arm.set_pose_reference_frame('panda_link0')

        # Slow down the max velocity and acceleration of the arm
        panda_arm.set_max_velocity_scaling_factor(0.2)
        panda_arm.set_max_acceleration_scaling_factor(0.2)

        group_name = "hand"
        hand = moveit_commander.MoveGroupCommander(group_name)

        # Slow down the max velocity and acceleration of the hand
        hand.set_max_velocity_scaling_factor(0.2)
        hand.set_max_acceleration_scaling_factor(0.2)

        #
        # TODO: it is likely that use of actionlib here is not necessary
        # MoveIt should be able to handle control of the gripper, however
        # I have not seen that working on rviz. This needs to be addressed
        # in the future.
        #
        gripper_homing_client = actionlib.SimpleActionClient(
            '/franka_gripper/homing', franka_gripper.msg.HomingAction)
        gripper_move_client = actionlib.SimpleActionClient(
            '/franka_gripper/move', franka_gripper.msg.MoveAction)
        gripper_grasp_client = actionlib.SimpleActionClient(
            '/franka_gripper/grasp', franka_gripper.msg.GraspAction)

        print "Waiting for gripper homing server"
        gripper_homing_client.wait_for_server()

        print "Waiting for gripper move server"
        gripper_move_client.wait_for_server()

        print "Waiting for gripper grasp server"
        gripper_grasp_client.wait_for_server()

        # We can get the name of the reference frame for this robot:
        print "============ Reference frame: ", panda_arm.get_planning_frame(), robot.get_planning_frame()

        eef_link = panda_arm.get_end_effector_link()
        print "============ End effector: %s" % eef_link

        group_names = robot.get_group_names()
        print "============ Robot Groups:", robot.get_group_names()

        print "============ Printing robot state"
        # print robot.get_current_state()
        print ""

        # panda_arm.remember_joint_values('home')
        # print "============ Printing bank of joints"
        # print panda_arm.get_remembered_joint_values()
        # print ""

        self.robot = robot
        self.panda_arm = panda_arm
        self.eef_link = eef_link
        self.group_names = group_names
        self.gripper_homing_client = gripper_homing_client
        self.gripper_move_client = gripper_move_client
        self.gripper_grasp_client = gripper_grasp_client

        self.O_P_EE = None   # current position of EE in link 0 frame
        self.O_V_EE = None   # current cartesian velocity of EE in link 0 frame
        self.O_P_EE_prev = None   # previous position of EE in link 0 frame
        self.O_V_EE_prev = None   # previous cartesian velocity of EE in link 0 frame

        self.O_P_EE_timestamp_secs = None
        self.O_P_EE_timestamp_secs_prev = None

        self.home_pose_joint_values = [0.02691706043507969,
                                       0.047798763040059276,
                                       -0.027120267404325524,
                                       -1.8725442233001977,
                                       -0.0055149872866853454,
                                       1.8590480223894117,
                                       0.7852137811630965]

        self.state_subscriber = rospy.Subscriber(
            '/franka_state_controller/franka_states',
            franka_msgs.msg.FrankaState,
            self.update_state)

    def update_state(self, msg):

        self.O_P_EE_prev = self.O_P_EE
        self.O_V_EE_prev = self.O_V_EE
        self.O_P_EE_timestamp_secs_prev = self.O_P_EE_timestamp_secs

        self.O_P_EE = np.array(msg.O_T_EE[-4:-1])
        self.O_P_EE_timestamp_secs = msg.header.stamp.secs + \
            msg.header.stamp.nsecs*(1e-9)

        if (self.O_P_EE_prev is not None):
            dt = self.O_P_EE_timestamp_secs - self.O_P_EE_timestamp_secs_prev
            assert (dt > 0)
            self.O_V_EE = (self.O_P_EE - self.O_P_EE_prev) / dt

        # Filter velocity (not sure if this is necessary)
        if (self.O_V_EE_prev is not None):
            self.O_V_EE = 0.5 * self.O_V_EE + 0.5*self.O_V_EE_prev

        #print (self.get_ee_state())

    def get_ee_state(self):
        return self.O_P_EE, self.O_V_EE

    def move_ee_to(self, pose_goal, wait_to_finish=True):
        # We can plan a motion for this group to a desired pose for the
        # end-effector:
        self.panda_arm.set_pose_target(pose_goal)

        # Now, we call the planner to compute the plan and execute it.
        plan = self.panda_arm.go(wait=wait_to_finish)
        # Calling `stop()` ensures that there is no residual movement
        self.panda_arm.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets()
        self.panda_arm.clear_pose_targets()

        current_pose = self.panda_arm.get_current_pose().pose
        return all_close(pose_goal, current_pose, 0.01)

    def shift_ee_by(self, axis=0, value=0.0, wait_to_finish=True):
        "axis in 0...5 for (x,y,z, r, p, y), value denotes the relative delta"
        self.panda_arm.shift_pose_target(axis, value)

        # Now, we call the planner to compute the plan and execute it.
        plan = self.panda_arm.go(wait=wait_to_finish)
        # Calling `stop()` ensures that there is no residual movement
        self.panda_arm.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets()
        self.panda_arm.clear_pose_targets()
        return True

    def move_joints_to(self, joints, wait_to_finish=True):
        # Now, we call the planner to compute the plan and execute it.
        plan = self.panda_arm.go(joints=joints, wait=wait_to_finish)
        # Calling `stop()` ensures that there is no residual movement
        self.panda_arm.stop()
        return True

    def actionlib_move_gripper_to(self, width, speed=0.05):
        goal = franka_gripper.msg.MoveGoal(width=width, speed=speed)
        self.gripper_move_client.send_goal(goal)
        self.gripper_move_client.wait_for_result()
        return self.gripper_move_client.get_result()

    def actionlib_gripper_homing(self):
        goal = franka_gripper.msg.HomingGoal()
        self.gripper_homing_client.send_goal(goal)
        self.gripper_homing_client.wait_for_result()
        return self.gripper_homing_client.get_result()

    def actionlib_gripper_grasp(self, width, speed=0.05, force=10):
        epsilon = franka_gripper.msg.GraspEpsilon(inner=0.0, outer=0.01)
        goal = franka_gripper.msg.GraspGoal(
            width=width, speed=speed, epsilon=epsilon, force=force)

        self.gripper_grasp_client.send_goal(goal)
        self.gripper_grasp_client.wait_for_result()
        return self.gripper_grasp_client.get_result()

    def moveit_move_gripper_to(self, pose_goal):
        # We can plan a motion for this group to a desired pose for the
        # end-effector:
        self.hand.set_pose_target(pose_goal)

        # Now, we call the planner to compute the plan and execute it.
        plan = self.hand.go(wait=True)
        # Calling `stop()` ensures that there is no residual movement
        self.hand.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets()
        self.hand.clear_pose_targets()

        # For testing:
        # Note that since this section of code will not be included in the tutorials
        # we use the class variable rather than the copied state variable
        current_pose = self.hand.get_current_pose().pose
        return all_close(pose_goal, current_pose, 0.01)


    def go_home(self):
        """Move the arm to a fixed valid position.
        """
        self.move_joints_to(self.home_pose_joint_values, wait_to_finish=True)
        
