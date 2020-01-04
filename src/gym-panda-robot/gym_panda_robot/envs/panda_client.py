#!/usr/bin/env python

import sys
import copy
import rospy

import actionlib
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import *

import franka_gripper
import franka_gripper.msg
from franka_msgs.msg import FrankaState
from franka_control.srv import SetFullCollisionBehavior
from franka_control.msg import ErrorRecoveryAction, ErrorRecoveryGoal

from controller_manager_msgs.srv import ListControllers
from controller_manager_msgs.srv import SwitchController

import moveit_commander
from moveit_commander.conversions import pose_to_list
import moveit_msgs.msg

import numpy as np
from math import pi


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

    elif isinstance(goal, PoseStamped):
        return all_close(goal.pose, actual.pose, tolerance)

    elif isinstance(goal, Pose):
        return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

    return True


class PandaClient(object):
    def __init__(self, moveit_safety_zone=[[-np.inf, np.inf], [-np.inf, np.inf], [-np.inf, np.inf]],
                 ee_safety_zone=[[-np.inf, np.inf], [-np.inf, np.inf], [-np.inf, np.inf]],
                 enable_kinect=True):
        
        super(PandaClient, self).__init__()

        self.moveit_safety_zone = moveit_safety_zone
        self.ee_safety_zone = ee_safety_zone
        
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
        
        panda_arm.set_workspace(ws=[moveit_safety_zone[0][0],
                                    moveit_safety_zone[1][0],
                                    moveit_safety_zone[2][0],
                                    moveit_safety_zone[0][1],
                                    moveit_safety_zone[1][1],
                                    moveit_safety_zone[2][1]])
                
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
        print robot.get_current_state()
        print ""

        # panda_arm.remember_joint_values('home')
        # print "============ Printing bank of joints"
        # print panda_arm.get_remembered_joint_values()
        # print ""

        self.robot = robot
        self.franka_state = None
        self.panda_arm = panda_arm
        self.eef_link = eef_link
        self.group_names = group_names
        self.gripper_homing_client = gripper_homing_client
        self.gripper_move_client = gripper_move_client
        self.gripper_grasp_client = gripper_grasp_client
        self.current_target_velocity = Twist()
        
        self.O_P_EE = None   # current position of EE in link 0 frame
        self.O_V_EE = None   # current cartesian velocity of EE in link 0 frame
        self.O_P_EE_prev = None   # previous position of EE in link 0 frame
        self.O_V_EE_prev = None   # previous cartesian velocity of EE in link 0 frame

        self.O_P_EE_timestamp_secs = None
        self.O_P_EE_timestamp_secs_prev = None
        self.depth_image = None
        self.rgb_image = None
        
        self.home_pose_joint_values = [0.02691706043507969,
                                       0.047798763040059276,
                                       -0.027120267404325524,
                                       -1.8725442233001977,
                                       -0.0055149872866853454,
                                       1.8590480223894117,
                                       0.7852137811630965]

        self.error_recovery_client = actionlib.SimpleActionClient('/franka_control/error_recovery',
                                                                  ErrorRecoveryAction)

        # Waits until the action server has started up and started
        # listening for goals.
        print ("Waiting for error recovery server in franka_control")
        self.error_recovery_client.wait_for_server()
        print ("Found error recovery server")

        self.cmd_vel_pub = rospy.Publisher("/franka_control/target_velocity", Twist, queue_size=1)
        self.target_vel_sub = rospy.Subscriber("/franka_control/current_target_velocity", Twist,
                                               self.target_vel_callback)

        self.state_subscriber = rospy.Subscriber('/franka_state_controller/franka_states',
                                                 FrankaState,
                                                 self.update_state)

        if enable_kinect:
            self.depth_subscriber = rospy.Subscriber('/depth/image_raw', Image, self.update_depth_image)
            self.rgb_subscriber = rospy.Subscriber('/rgb/image_raw', Image, self.update_rgb_image)

    def update_depth_image(self, data):
        self.depth_image = data

    def update_rgb_image(self, data):
        self.rgb_image = data
        
    def ee_inside_safety_zone(self, xyz):
        return xyz[0] >= self.ee_safety_zone[0][0] and xyz[0] <= self.ee_safety_zone[0][1] and \
            xyz[1] >= self.ee_safety_zone[1][0] and xyz[1] <= self.ee_safety_zone[1][1] and \
            xyz[2] >= self.ee_safety_zone[2][0] and xyz[2] <= self.ee_safety_zone[2][1]
        
    def update_state(self, msg):

        self.O_P_EE_prev = self.O_P_EE
        self.O_V_EE_prev = self.O_V_EE
        self.O_P_EE_timestamp_secs_prev = self.O_P_EE_timestamp_secs

        self.O_P_EE = np.array(msg.O_T_EE[-4:-1])
        self.O_P_EE_timestamp_secs = msg.header.stamp.secs + msg.header.stamp.nsecs*(1e-9)

        if self.O_P_EE_prev is None and not self.ee_inside_safety_zone(self.O_P_EE):
            raise Exception('Initial position {} should be inside the safety zone {}'.format(self.O_P_EE,
                                                                                             self.ee_safety_zone))
        
        if (self.O_P_EE_prev is not None):
            dt = self.O_P_EE_timestamp_secs - self.O_P_EE_timestamp_secs_prev
            assert (dt > 0)
            self.O_V_EE = (self.O_P_EE - self.O_P_EE_prev) / dt

        # Filter velocity (not sure if this is necessary)
        if (self.O_V_EE_prev is not None):
            self.O_V_EE = 0.5 * self.O_V_EE + 0.5*self.O_V_EE_prev

        self.franka_state = msg
        #print (self.get_ee_state())

    def target_vel_callback(self, twist):
        self.current_target_velocity = twist
        
    def get_ee_state(self):
        return self.O_P_EE, self.O_V_EE

    def move_ee_to(self, pose_goal, wait_to_finish=True):

        if not self.ee_inside_safety_zone(pose_goal):
            raise Exception('Goal ee pose should be inside the safety zone {}'.format(pose_goal,
                                                                                      self.ee_safety_zone))
        
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

        dP = np.array([0.0, 0.0, 0.0])
        dP[axis] = value
        req = self.O_P_EE + dP
        if not self.ee_inside_safety_zone(req):
            raise Exception('Shifted ee pose should be inside the safety zone {}'.format(req,
                                                                                         self.ee_safety_zone))
        
        "axis in 0...5 for (x,y,z, r, p, y), value denotes the relative delta"
        self.panda_arm.shift_pose_target(axis, value)

        # Now, we call the planner to compute the plan and execute it.
        plan = self.panda_arm.go(wait=wait_to_finish)
        # Calling `stop()` ensures that there is no residual movement
        self.panda_arm.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets()
        self.panda_arm.clear_pose_targets()

        if not running:
            print ("Switching to velocity control")
            self.switch_controllers('position_joint_trajectory_controller',
                                    'cartesian_velocity_example_controller')

        return True

    def move_joints_to(self, joints, wait_to_finish=True):
        # Now, we call the planner to compute the plan and execute it.
        plan = self.panda_arm.go(joints=joints, wait=wait_to_finish)
        # Calling `stop()` ensures that there is no residual movement
        self.panda_arm.stop()
        return True

    def move_gripper_to(self, width, speed=0.05):
        goal = franka_gripper.msg.MoveGoal(width=width, speed=speed)
        self.gripper_move_client.send_goal(goal)
        self.gripper_move_client.wait_for_result()
        return self.gripper_move_client.get_result()

    def gripper_homing(self):
        goal = franka_gripper.msg.HomingGoal()
        self.gripper_homing_client.send_goal(goal)
        self.gripper_homing_client.wait_for_result()
        return self.gripper_homing_client.get_result()

    def gripper_grasp(self, width, speed=0.05, force=10):
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


    def set_ee_velocity(self, twist):

        if self.O_P_EE is None:
            return
        
        #
        # Clip x-velocity if outsize the safety zone and request is to further violate it
        #
        if self.O_P_EE[0] < self.ee_safety_zone[0][0] and twist.linear.x < 0:
            twist.linear.x = 0.0
        
        if self.O_P_EE[0] > self.ee_safety_zone[0][1] and twist.linear.x > 0:
            twist.linear.x = 0.0

        #
        # Clip y-velocity if outsize the safety zone and request is to further violate it
        #
        if self.O_P_EE[1] < self.ee_safety_zone[1][0] and twist.linear.y < 0:
            twist.linear.y = 0.0
        
        if self.O_P_EE[1] > self.ee_safety_zone[1][1] and twist.linear.y > 0:
            twist.linear.y = 0.0


        #
        # Clip z-velocity if outsize the safety zone and request is to further violate it
        #
        if self.O_P_EE[2] < self.ee_safety_zone[2][0] and twist.linear.z < 0:
            twist.linear.z = 0.0
        
        if self.O_P_EE[2] > self.ee_safety_zone[2][1] and twist.linear.z > 0:
            twist.linear.z = 0.0
           
        self.cmd_vel_pub.publish(twist)
        
    def recover_from_errors(self):
        goal = ErrorRecoveryGoal()
        self.error_recovery_client.send_goal(goal)
        #print ("Waiting for recovery goal")
        #self.error_recovery_client.wait_for_result()
        #print ("Done")
        return self.error_recovery_client.get_result()
    
        
    def go_home(self):
        """Move the arm to a fixed home position.
        """
        running = self.controller_is_running('position_joint_trajectory_controller')

        if not running:
            print ("Switching to position control")
            rospy.sleep(3)
            resp = self.switch_controllers('cartesian_velocity_example_controller',
                                           'position_joint_trajectory_controller')

        self.move_joints_to(self.home_pose_joint_values, wait_to_finish=True)
        
        if not running:
            print ("Switching to velocity control")
            self.switch_controllers('position_joint_trajectory_controller',
                                    'cartesian_velocity_example_controller')


    def switch_controllers(self, stop_controller, start_controller):
        rospy.wait_for_service('/controller_manager/switch_controller')
        try:
            srv = rospy.ServiceProxy('/controller_manager/switch_controller', SwitchController)
            req = {'start_controllers': [start_controller],
                   'stop_controllers': [stop_controller],
                   'strictness': 2,  # 2=strict, 1=best effort
                    #'start_asap': True, # for some reason this was excluded from ROS melodic
                    #'timeout': 3,  # in seconds
            } 
            
            resp = srv(**req)
            return resp
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

        
    def controller_is_running(self, name):
        controller_status = self.query_controller_status()
        for cs in controller_status.controller:
            if cs.name == name:
                return cs.state == 'running'

        return False
        
    def query_controller_status(self):
        rospy.wait_for_service('/controller_manager/list_controllers')
        try:
            srv = rospy.ServiceProxy('/controller_manager/list_controllers', ListControllers)
            resp = srv()
            return resp
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e


    def set_collision_behavior(self):
        rospy.wait_for_service('/franka_control/set_full_collision_behavior')

        cb = {
            'lower_torque_thresholds_acceleration': [120.0, 120.0, 118.0, 118.0, 116.0, 114.0, 112.0],
            'upper_torque_thresholds_acceleration': [120.0, 120.0, 118.0, 118.0, 116.0, 114.0, 112.0],
            'lower_torque_thresholds_nominal': [120.0, 120.0, 118.0, 118.0, 116.0, 114.0, 112.0],
            'upper_torque_thresholds_nominal': [120.0, 120.0, 118.0, 118.0, 116.0, 114.0, 112.0], 
            'lower_force_thresholds_acceleration': [120.0, 120.0, 120.0, 125.0, 125.0, 125.0],
            'upper_force_thresholds_acceleration': [120.0, 120.0, 120.0, 125.0, 125.0, 125.0],
            'lower_force_thresholds_nominal': [120.0, 120.0, 120.0, 125.0, 125.0, 125.0],
            'upper_force_thresholds_nominal': [120.0, 120.0, 120.0, 125.0, 125.0, 125.0]
        }

        for k,v in cb.items():
            cb[k] = [vi*0.1 for vi in v]
            if k.startswith('upper'):
                cb[k] = [vi for vi in v]

        try:
            collision_behavior_srv = rospy.ServiceProxy('/franka_control/set_full_collision_behavior',
                                                        SetFullCollisionBehavior)
            resp1 = collision_behavior_srv(**cb)
            print (resp1)
            return resp1.success
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    def is_in_collision_mode(self):
        if self.franka_state is None:
            return False

        return self.franka_state.robot_mode == FrankaState.ROBOT_MODE_REFLEX
    
    def is_in_move_mode(self):
        if self.franka_state is None:
            return False
        
        return self.franka_state.robot_mode == FrankaState.ROBOT_MODE_MOVE
    
    def is_in_idle_mode(self):
        if self.franka_state is None:
            return False
        
        return self.franka_state.robot_mode == FrankaState.ROBOT_MODE_IDLE
    
