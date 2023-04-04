#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2013, SRI International
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of SRI International nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Acorn Pooley, Mike Lautman

# Inspired from http://docs.ros.org/kinetic/api/moveit_tutorials/html/doc/move_group_python_interface/move_group_python_interface_tutorial.html
# Modified by Alexandre Vannobel to test the FollowJointTrajectory Action Server for the Kinova Gen3 robot

# To run this node in a given namespace with rosrun (for example 'my_gen3'), start a Kortex driver and then run : 
# rosrun kortex_examples example_moveit_trajectories.py __ns:=my_gen3

from geometry_msgs.msg import PoseStamped, PoseArray
from kortex_driver.srv import *
from kortex_driver.msg import *
from moveit_msgs.msg import *
from tf.transformations import euler_from_quaternion, quaternion_from_euler

import actionlib

import sys
import time
import rospy
import moveit_commander


import math
import time


import numpy as np


from math import pi
from std_srvs.srv import Empty
from std_msgs.msg import Empty, Int16

class ExampleMoveItTrajectories(object):
  """ExampleMoveItTrajectories"""
  def __init__(self):

    # Initialize the node
    super(ExampleMoveItTrajectories, self).__init__()
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('example_move_it_trajectories')

    self.HOME_ACTION_IDENTIFIER = 2
    self.REST_ACTION_IDENTIFIER = 1

    self.FirstTrajectory = RobotTrajectory()
    self.SecondTrajectory = RobotTrajectory()

    try:
      self.is_gripper_present = rospy.get_param(rospy.get_namespace() + "is_gripper_present", False)
      if self.is_gripper_present:
        gripper_joint_names = rospy.get_param(rospy.get_namespace() + "gripper_joint_names", [])
        self.gripper_joint_name = gripper_joint_names[0]
      else:
        gripper_joint_name = ""
      self.degrees_of_freedom = rospy.get_param(rospy.get_namespace() + "degrees_of_freedom", 7)

      # Create the MoveItInterface necessary objects
      arm_group_name = "arm"
      self.robot = moveit_commander.RobotCommander("robot_description")
      self.scene = moveit_commander.PlanningSceneInterface(ns=rospy.get_namespace())
      self.arm_group = moveit_commander.MoveGroupCommander(arm_group_name, ns=rospy.get_namespace())
      self.display_trajectory_publisher = rospy.Publisher(rospy.get_namespace() + 'move_group/display_planned_path',
                                                    moveit_msgs.msg.DisplayTrajectory,
                                                    queue_size=20)
      self.main_plan = self.arm_group.plan()

      # Create the second MoveItInterface necessary objects
      arm_group_name2 = "arm"
      self.robot2 = moveit_commander.RobotCommander("robot_description")
      self.scene2 = moveit_commander.PlanningSceneInterface(ns=rospy.get_namespace())
      self.arm_group2 = moveit_commander.MoveGroupCommander(arm_group_name2, ns=rospy.get_namespace())
      self.display_trajectory_publisher2 = rospy.Publisher(rospy.get_namespace() + 'move_group/SecondTrajectory',
                                                    moveit_msgs.msg.DisplayTrajectory,
                                                    queue_size=20)
      self.main_plan2 = self.arm_group2.plan()




      if self.is_gripper_present:
        gripper_group_name = "gripper"
        self.gripper_group = moveit_commander.MoveGroupCommander(gripper_group_name, ns=rospy.get_namespace())

      rospy.loginfo("Initializing node in namespace " + rospy.get_namespace())

      self.robot_name = rospy.get_param('~robot_name', "my_gen3_lite")

      # Init the services
      clear_faults_full_name = '/' + self.robot_name + '/base/clear_faults'
      rospy.wait_for_service(clear_faults_full_name)
      self.clear_faults = rospy.ServiceProxy(clear_faults_full_name, Base_ClearFaults)

      read_action_full_name = '/' + self.robot_name + '/base/read_action'
      rospy.wait_for_service(read_action_full_name)
      self.read_action = rospy.ServiceProxy(read_action_full_name, ReadAction)

      execute_action_full_name = '/' + self.robot_name + '/base/execute_action'
      rospy.wait_for_service(execute_action_full_name)
      self.execute_action = rospy.ServiceProxy(execute_action_full_name, ExecuteAction)

      activate_publishing_of_action_notification_full_name = '/' + self.robot_name + '/base/activate_publishing_of_action_topic'
      rospy.wait_for_service(activate_publishing_of_action_notification_full_name)
      self.activate_publishing_of_action_notification = rospy.ServiceProxy(activate_publishing_of_action_notification_full_name, OnNotificationActionTopic)
  
      get_product_configuration_full_name = '/' + self.robot_name + '/base/get_product_configuration'
      rospy.wait_for_service(get_product_configuration_full_name)
      self.get_product_configuration = rospy.ServiceProxy(get_product_configuration_full_name, GetProductConfiguration)

      self.trajectory_execution_sub = rospy.Subscriber("/KinovaAR/execute_action", Empty, self.trajectory_execution_callback)
      self.target_pose_sub = rospy.Subscriber("/KinovaAR/targetPose", PoseStamped, self.target_pose_callback)
      self.waypoint_pose_sub = rospy.Subscriber("/KinovaAR/WayPoints", PoseArray, self.waypoint_callback)

      self.GripperSubscriber = rospy.Subscriber("/KinovaAR/Pose", Int16, self.KinovaPose_callback)
      self.RL_Home_Position_Subscriber = rospy.Subscriber("/KinovaAR/RL_HomePosition", Empty, self.reach_home_joint_values)

      # self.trajectory_result_sub = rospy.Subscriber("/" + self.robot_name + "/execute_trajectory/result", ExecuteTrajectoryActionResult, self.trajectory_result_callback)
      self.execute_sequence_sub = rospy.Subscriber("/KinovaAR/execute_sequence", Empty, self.execute_sequence_callback)

      self.trajectorySequence_Publisher = rospy.Publisher("/KinovaAR/execute_practice_sequence", Empty, queue_size=1)

      self.reset_position_sub = rospy.Subscriber("/KinovaAR/reset_position", Empty, self.reset_position_callback)
      self.reset_position_reached_pub = rospy.Publisher("/KinovaAR/reset_position/reached", Empty, queue_size=1)
      self.trajectory_position_reached_pub = rospy.Publisher("/KinovaAR/trajectory_position/reached", Empty, queue_size=1)


      self.load_FirstTrajectory_sub = rospy.Subscriber("/KinovaAR/FirstTrajectory", DisplayTrajectory, self.loadFirstTrajectory)
      self.load_SecondTrajectory_sub = rospy.Subscriber("/KinovaAR/SecondTrajectory", DisplayTrajectory, self.loadSecondTrajectory)
      self.execute_sequence_sub = rospy.Subscriber("/KinovaAR/execute_FirstTrajectory", Empty, self.playFirstTrajectory)
      self.execute_sequence_sub = rospy.Subscriber("/KinovaAR/execute_SecondTrajectory", Empty, self.playSecondTrajectory)

      self.firstTrajectory_sub = rospy.Subscriber("/KinovaAR/FirstTrajectory", DisplayTrajectory, self.concat)
      self.concat_pub = rospy.Publisher("/KinovaAR/ConcatTrajectory", DisplayTrajectory, queue_size=1)

    except Exception as e:   
      print (e)
      self.is_init_success = False
    else:
      self.is_init_success = True

    success = self.is_init_success

    if success:    
      rospy.loginfo("Printing current joint values :")
      self.get_current_joint_values()
     
      # rospy.loginfo("Printing current position :")
      # self.get_cartesian_pose()
      self.reach_gripper_position(0.5)
      self.example_rest_the_robot()
      # self.reach_home_joint_values()
      # self.get_cartesian_pose()
      # self.example_cartesian_waypoint_action()
      # self.reach_named_position("vertical")

    try:
      rospy.spin()
    except:
      rospy.logerr("Failed to call ROS spin")

  def concat(self, data):
      second_msg = rospy.wait_for_message("/KinovaAR/FirstTrajectory", DisplayTrajectory)
      
      appended_msg = DisplayTrajectory()
      appended_msg.trajectory = data.trajectory + second_msg.trajectory
      appended_msg.RobotState = data.RobotState
      self.concat_pub.publish(appended_msg)

      print("Printed Concated message")



  def waypoint_callback(self, data):
    
    req = ExecuteActionRequest()

    trajectory = WaypointList()

    pos_x1 = data.poses[0].position.x
    pos_y1 = data.poses[0].position.y
    pos_z1 = data.poses[0].position.z
    pos_1quat1 = data.poses[0].orientation.x
    pos_1quat2 = data.poses[0].orientation.y
    pos_1quat3 = data.poses[0].orientation.z
    pos_1quat4 = data.poses[0].orientation.w

    pos_x2 = data.poses[1].position.x
    pos_y2 = data.poses[1].position.y
    pos_z2 = data.poses[1].position.z
    pos_2quat1 = data.poses[1].orientation.x
    pos_2quat2 = data.poses[1].orientation.y
    pos_2quat3 = data.poses[1].orientation.z
    pos_2quat4 = data.poses[1].orientation.w

    orientation_list = [pos_1quat1, pos_1quat2, pos_1quat3, pos_1quat4]
    (roll1, pitch1, yaw1) = euler_from_quaternion (orientation_list)

    orientation_list = [pos_2quat1, pos_2quat2, pos_2quat3, pos_2quat4]
    (roll2, pitch2, yaw2) = euler_from_quaternion (orientation_list)

    trajectory.waypoints.append(self.FillCartesianWaypoint(pos_x1, pos_y1, pos_z1, roll1, pitch1, yaw1, 0))
    trajectory.waypoints.append(self.FillCartesianWaypoint(pos_x2, pos_y2, pos_z2, roll2, pitch2, yaw2, 0))

    req.input.oneof_action_parameters.execute_waypoint_list.append(trajectory)
    try:
        self.execute_action(req)
    except rospy.ServiceException:
        rospy.logerr("Failed to call action ExecuteWaypointTrajectory")
        return False
    else:
        return self.wait_for_action_end_or_abort()

    # success = True

    # self.reach_cartesian_pose(pose=data.poses[0], tolerance=0.01, constraints=None)
    # suxccess &= self.trajectory_execution_callback()
    
    # success &= self.reach_cartesian_pose(pose=data.poses[1], tolerance=0.01, constraints=None)
    # success &= self.trajectory_execution_callback()

    # for i in waypoint_count:
       



  def loadFirstTrajectory(self, data):
    self.FirstTrajectory = data.trajectory[0]
    rospy.loginfo("First Trajectory Logged")

  def loadSecondTrajectory(self, data):
    self.SecondTrajectory = data.trajectory[0]
    rospy.loginfo("Second Trajectory Logged")

  def playFirstTrajectory(self, data):
    
    sequence = True
    self.main_plan = self.FirstTrajectory
    try:
      sequence &= self.arm_group.execute(self.main_plan, wait=True)
        # self.execute_action(req)
    except rospy.ServiceException:
        rospy.logerr("Failed to call ExecuteAction for First Trajectory")
        # return False
    else:
      # sequence &= self.PointAndReturn()
      pass

  def playSecondTrajectory(self, data):
    sequence = True

    self.main_plan = self.SecondTrajectory
    try:
      sequence &= self.arm_group.execute(self.main_plan, wait=True)
        # self.execute_action(req)
    except rospy.ServiceException:
        rospy.logerr("Failed to call ExecuteAction for Second Trajectory")
        # return False
    else:
      # sequence &= self.wait_for_action_end_or_abort()
      sequence &= self.PointAndReturn()
  
  def example_home_the_robot(self):

    self.last_action_notif_type = None
    req = ReadActionRequest()
    req.input.identifier = self.HOME_ACTION_IDENTIFIER
    try:
        res = self.read_action(req)
    except rospy.ServiceException:
        rospy.logerr("Failed to call ReadAction")
        return False
    # Execute the HOME action if we could read it
    else:
        # What we just read is the input of the ExecuteAction service
        req = ExecuteActionRequest()
        req.input = res.output
        rospy.loginfo("Sending the robot home...")
        try:
            self.execute_action(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call ExecuteAction")
            return False
        else:
            return self.wait_for_action_end_or_abort()

  def example_rest_the_robot(self):
    self.last_action_notif_type = None
    req = ReadActionRequest()
    req.input.identifier = self.REST_ACTION_IDENTIFIER
    try:
        res = self.read_action(req)
    except rospy.ServiceException:
        rospy.logerr("Failed to call ReadAction")
        return False
    else:
        # What we just read is the input of the ExecuteAction service
        req = ExecuteActionRequest()
        req.input = res.output
        rospy.loginfo("Sending robot to rest position...")
        try:
            self.execute_action(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call ExecuteAction")
            return False
        else:
            self.reset_position_reached_pub.publish(Empty())
            pass
            # return self.wait_for_action_end_or_abort()

  def wait_for_action_end_or_abort(self):
    while not rospy.is_shutdown():
        if (self.last_action_notif_type == ActionEvent.ACTION_END):
            rospy.loginfo("Received ACTION_END notification")
            return True
        elif (self.last_action_notif_type == ActionEvent.ACTION_ABORT):
            rospy.loginfo("Received ACTION_ABORT notification")
            return False
        else:
            time.sleep(0.01)

  def reset_position_callback(self, data):
    rospy.loginfo("Reseting Position!")
    self.reach_gripper_position(0.5)
    self.example_rest_the_robot()

  def test_sequence(self):
    arm_group = self.arm_group
    joint_positions = arm_group.get_current_joint_values()

    joint_positions[0] = 1.836
    joint_positions[1] = -0.129
    joint_positions[2] = 2.063
    joint_positions[3] = -1.587
    joint_positions[4] = -0.941
    joint_positions[5] = 0.313

    arm_group.set_joint_value_target(joint_positions)

    try:
      # Plan the new trajectory
      self.main_plan = arm_group.plan()
    except:
      rospy.logerr("Failed to plan trajectory.")
      # Call function to reset the position of target pose object to end effector location

    else:
      rospy.loginfo("Planned Initial Pose!")



  def reach_dropoff_joint_values(self):
    arm_group = self.arm_group
    joint_positions = arm_group.get_current_joint_values()

    joint_positions[0] = 1.836
    joint_positions[1] = -0.129
    joint_positions[2] = 2.063
    joint_positions[3] = -1.587
    joint_positions[4] = -0.941
    joint_positions[5] = 0.313

    arm_group.set_joint_value_target(joint_positions)

    try:
      # Plan the new trajectory
      self.main_plan = arm_group.plan()
      print(arm_group.plan())
    except:
      rospy.logerr("Failed to plan trajectory.")
      # Call function to reset the position of target pose object to end effector location

    else:
      rospy.loginfo("Planned Initial Pose!")
      arm_group.go(wait=True)



  def reach_home_joint_values(self):
    arm_group = self.arm_group
    joint_positions = arm_group.get_current_joint_values()

    joint_positions[0] = 1.836
    joint_positions[1] = 0.033
    joint_positions[2] = 1.57
    joint_positions[3] = -1.587
    joint_positions[4] = -1.59
    joint_positions[5] = 0.304

    arm_group.set_joint_value_target(joint_positions)

    try:
      # Plan the new trajectory
      self.main_plan = arm_group.plan()
      print(arm_group.plan())
    except:
      rospy.logerr("Failed to plan trajectory.")
      # Call function to reset the position of target pose object to end effector location

    else:
      rospy.loginfo("Planned Initial Pose!")
      arm_group.go(wait=True)

  def get_current_joint_values(self):
    arm_group = self.arm_group
    joint_positions = arm_group.get_current_joint_values()
    for p in joint_positions: rospy.loginfo(p)

  def KinovaPose_callback(self, data):
        self.last_action_notif_type = None
        req = ReadActionRequest()
        
        req.input.identifier = data.data

        try:
            res = self.read_action(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call ReadAction")
            return False
        # Execute the HOME action if we could read it
        else:
            # What we just read is the input of the ExecuteAction service
            req = ExecuteActionRequest()
            req.input = res.output
            # rospy.loginfo("Sending the robot home...")
            try:
                self.execute_action(req)
            except rospy.ServiceException:
                rospy.logerr("Failed to call ExecuteAction")
                return False
            else:
                return self.wait_for_action_end_or_abort()

  def target_pose_callback(self, data):
    rospy.loginfo(data.pose.position)

    self.reach_cartesian_pose(pose=data.pose, tolerance=0.01, constraints=None)

  def get_euler_from_quaterion(self, x,y,z,w):
     t0 


  def get_quaternion_from_euler(self, roll, pitch, yaw):
    """
    Convert an Euler angle to a quaternion.
     
    Input
      :param roll: The roll (rotation around x-axis) angle in radians.
      :param pitch: The pitch (rotation around y-axis) angle in radians.
      :param yaw: The yaw (rotation around z-axis) angle in radians.
   
    Output
      :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
    """
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
   
    return [qx, qy, qz, qw]



  # def example_cartesian_waypoint_action(self):
  #     self.last_action_notif_type = None

  #     client = actionlib.SimpleActionClient('/' + self.robot_name + '/cartesian_trajectory_controller/follow_cartesian_trajectory', kortex_driver.msg.FollowCartesianTrajectoryAction)
  #     client.wait_for_server()
  #     goal = FollowCartesianTrajectoryGoal()

  #     config = self.get_product_configuration()

  #     # Crane positions for Kinova Arm
  #     goal.trajectory.append(self.FillCartesianWaypoint(0.17,  0.20,  0.14, math.radians(3.3), math.radians(180), math.radians(90), 0))

  #     quat = self.get_quaternion_from_euler(math.radians(3.3), math.radians(180), math.radians(90))

  #     pose = geometry_msgs.msg.Pose()

  #     pose.position.x = 0.17
  #     pose.position.y = 0.2
  #     pose.position.z = 0.14
  #     pose.orientation.x = quat[0]
  #     pose.orientation.y = quat[1]
  #     pose.orientation.z = quat[2]
  #     pose.orientation.w = quat[3]

  #     self.reach_cartesian_pose(pose=pose, tolerance=0.01, constraints=None)


  def FillCartesianWaypoint(self, new_x, new_y, new_z, new_theta_x, new_theta_y, new_theta_z, blending_radius):
      cartesianWaypoint = CartesianWaypoint()

      cartesianWaypoint.pose.x = new_x
      cartesianWaypoint.pose.y = new_y
      cartesianWaypoint.pose.z = new_z
      cartesianWaypoint.pose.theta_x = new_theta_x
      cartesianWaypoint.pose.theta_y = new_theta_y
      cartesianWaypoint.pose.theta_z = new_theta_z
      cartesianWaypoint.reference_frame = CartesianReferenceFrame.CARTESIAN_REFERENCE_FRAME_BASE
      cartesianWaypoint.blending_radius = blending_radius
     
      return cartesianWaypoint

  def reach_named_position(self, target):
    arm_group = self.arm_group
    
    # Going to one of those targets
    rospy.loginfo("Going to named target " + target)
    # Set the target
    arm_group.set_named_target(target)
    # Plan the trajectory
    planned_path1 = arm_group.plan()

    # Execute the trajectory and block while it's not finished
    # return arm_group.execute(planned_path1, wait=True)
    return True

  def reach_joint_angles(self, tolerance):
    arm_group = self.arm_group
    success = True

    # Get the current joint positions
    joint_positions = arm_group.get_current_joint_values()
    rospy.loginfo("Printing current joint positions before movement :")
    for p in joint_positions: rospy.loginfo(p)

    # Set the goal joint tolerance
    self.arm_group.set_goal_joint_tolerance(tolerance)

    # Set the joint target configuration
    if self.degrees_of_freedom == 7:
      joint_positions[0] = pi/2
      joint_positions[1] = 0
      joint_positions[2] = pi/4
      joint_positions[3] = -pi/4
      joint_positions[4] = 0
      joint_positions[5] = pi/2
      joint_positions[6] = 0.2
    elif self.degrees_of_freedom == 6:
      joint_positions[0] = 0
      joint_positions[1] = 0
      joint_positions[2] = pi/2
      joint_positions[3] = pi/4
      joint_positions[4] = 0
      joint_positions[5] = pi/2
    arm_group.set_joint_value_target(joint_positions)
    
    # Plan and execute in one command
    success &= arm_group.go(wait=True)

    # Show joint positions after movement
    new_joint_positions = arm_group.get_current_joint_values()
    rospy.loginfo("Printing current joint positions after movement :")
    for p in new_joint_positions: rospy.loginfo(p)
    return success

  def get_cartesian_pose(self):
    arm_group = self.arm_group

    # Get the current pose and display it
    pose = arm_group.get_current_pose()
    rospy.loginfo("Actual cartesian pose is : ")
    rospy.loginfo(pose.pose)

    return pose.pose

  def reach_cartesian_pose(self, pose, tolerance, constraints):
    arm_group = self.arm_group
    
    # Set the tolerance
    arm_group.set_goal_position_tolerance(tolerance)

    # Set the trajectory constraint if one is specified
    if constraints is not None:
      arm_group.set_path_constraints(constraints)

    # Get the current Cartesian Position
    arm_group.set_pose_target(pose)

    try:
      # Plan the new trajectory
      self.main_plan = arm_group.plan()
      print(arm_group.plan())
    except:
      rospy.logerr("Failed to plan trajectory.")
      # Call function to reset the position of target pose object to end effector location

    else:
      pass      

    self.arm_group = arm_group
    # Plan

    rospy.loginfo("Planning")

    # return arm_group.go(wait=True)


  def trajectory_sequence_callback(self, data):
    self.last_action_notif_type = None

    try:
      self.arm_group.execute(self.main_plan, wait=True)
        # self.execute_action(req)
    except rospy.ServiceException:
        rospy.logerr("Failed to call ExecuteAction")
        # return False
    else:
      self.capture_the_flag()


  def trajectory_execution_callback(self, data):
    self.last_action_notif_type = None

    try:
      self.arm_group.execute(self.main_plan, wait=True)
        # self.execute_action(req)
    except rospy.ServiceException:
        rospy.logerr("Failed to call ExecuteAction")
        # return False
    else:
      return self.wait_for_action_end_or_abort()

      


  def trajectory_result_callback(self, data):
    
    status_value = data.result.error_code.val
    
    if(status_value == 1):
      rospy.loginfo("success!!!!")

      # Call the sequence 
      sequence_message = Empty()
      # self.trajectorySequence_Publisher.publish(sequence_message)
    
    elif(status_value == -4):
      rospy.loginfo("Solution found but controller failed...")
    

  def reach_gripper_position(self, relative_position):
    gripper_group = self.gripper_group
    
    # We only have to move this joint because it is symmetrical!
    gripper_joint = self.robot.get_joint(self.gripper_joint_name)
    gripper_max_absolute_pos = gripper_joint.max_bound()
    gripper_min_absolute_pos = gripper_joint.min_bound()
    
    try:
      self.main_plan = gripper_group.plan()
      rospy.loginfo("Planning Gripper")
    except:
      rospy.loginfo("Failed to plan gripper")
      return False 
    else:  
      gripper_joint.move(relative_position * (gripper_max_absolute_pos - gripper_min_absolute_pos) + gripper_min_absolute_pos, True)

  def execute_sequence_callback(self, data):
    # self.test_sequence()
    self.capture_the_flag()

  def capture_the_flag(self):
     
    self.reach_gripper_position(0.01)
    self.reach_dropoff_joint_values()
    self.reach_gripper_position(0.5)
    self.reach_home_joint_values()

  def PointAndReturn(self):
    self.trajectory_position_reached_pub.publish(Empty())
    self.reach_gripper_position(0.01)
    self.reach_gripper_position(0.5)
    self.example_rest_the_robot()




if __name__ == '__main__':
  example = ExampleMoveItTrajectories()

