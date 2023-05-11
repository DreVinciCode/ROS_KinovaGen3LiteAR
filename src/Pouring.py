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

from geometry_msgs.msg import PoseStamped, PoseArray, Pose as ps
from kortex_driver.srv import *
from kortex_driver.msg import *
from moveit_msgs.msg import *
from kinova_study.msg import load_trajectory
from sensor_msgs.msg import JointState

# from tf.transformations import euler_from_quaternion, quaternion_from_euler

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
from std_msgs.msg import Empty, Int16, Header

class ExampleMoveItTrajectories(object):
  """ExampleMoveItTrajectories"""
  def __init__(self):

    # Initialize the node
    super(ExampleMoveItTrajectories, self).__init__()
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('example_move_it_trajectories')

    self.HOME_ACTION_IDENTIFIER = 2
    self.REST_ACTION_IDENTIFIER = 1

    self.i_limit = 2147483647

    self.appended_msg = DisplayTrajectory() 
    self.plan_array = []
    
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



    # Set own subscribers and publishers
      self.execute_action_sub = rospy.Subscriber("/KinovaAR/execute_action", Empty, self.execute_action_callback)

      self.display_trajectory_pub  = rospy.Publisher("/KinovaAR/FirstTrajectoryDisplay", DisplayTrajectory, queue_size=1)

    

    except Exception as e:   
      print (e)
      self.is_init_success = False
    else:
      self.is_init_success = True

    success = self.is_init_success

    if success:    
        # self.reach_pourhome_joint_values()
        # self.get_current_joint_values()
        self.example_home_the_robot()
        # self.reach_home_joint_values()    #
        # self.move_with_pose()
        self.setPosesTarget()
    else:
       print("Failed")

    try:
      rospy.spin()
    except:
      rospy.logerr("Failed to call ROS spin")

  def get_quaternion_from_euler(self, roll, pitch, yaw):
    """
    Convert an Euler angle to a quaternion.
     
    Input
      :param roll: The roll (rotation around x-axis) angle in radians.
      :param pitch: The pitch (rotation around y-axis) angle in radians.
      :param yaw: The yaw (rotation around z-axis) angle in radians.
   
    Output
      :return qx, qy, qz, qw: The orientation in quaernion [x,y,z,w] format
    """
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

    return [qx,qy,qz,qw]

  def ConvertToQuat(self, ori):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    x = ori.x
    y = ori.y
    z = ori.z
    w = ori.w

    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z # in radians


  def setPosesTarget(self):

    arm_group = self.arm_group
    self.arm_group.set_max_velocity_scaling_factor(0.3)
    currentPose = arm_group.get_current_pose()
    ori = currentPose.pose.orientation

    roll, pitch, yaw = self.ConvertToQuat(ori)


    poses = []

    pose0 = currentPose.pose
    poses.append(pose0)

    pose1 = pose0
    pose1.position.x = pose0.position.x - 0.075
    poses.append(pose1)


    arm_group.set_pose_targets(poses)
    arm_group.plan()

    plan = rospy.wait_for_message("/my_gen3_lite/move_group/display_planned_path", DisplayTrajectory)
    self.plan_array.append(plan)
    
    joint_state = JointState()
    joint_state.header = Header()
    joint_state.header.stamp = rospy.Time.now()
    joint_state.name = plan.trajectory[0].joint_trajectory.joint_names
    joint_state.position = plan.trajectory[0].joint_trajectory.points[-1].positions   
    moveit_robot_state = RobotState()
    moveit_robot_state.joint_state = joint_state
    arm_group.set_start_state(moveit_robot_state)
 

    joint_positions = joint_state.position

    pour_joints = arm_group.get_current_joint_values()

    pour_joints[0] = joint_positions[0]
    pour_joints[1] = joint_positions[1]
    pour_joints[2] = joint_positions[2]
    pour_joints[3] = joint_positions[3]
    pour_joints[4] = joint_positions[4]
    pour_joints[5] = -2.59

    arm_group.set_joint_value_target(pour_joints)
    arm_group.plan()

    plan = rospy.wait_for_message("/my_gen3_lite/move_group/display_planned_path", DisplayTrajectory)
    self.plan_array.append(plan)
    
    self.concat_trajectories(self.plan_array)


  def concat_trajectories(self, trajectories):
    self.appended_msg = DisplayTrajectory()
    self.appended_msg.model_id = trajectories[0].model_id
    self.appended_msg.trajectory = trajectories[0].trajectory
    self.appended_msg.trajectory_start = trajectories[0].trajectory_start                

    for plan in range(1, len(trajectories)):
        concat_seconds = self.appended_msg.trajectory[0].joint_trajectory.points[-1].time_from_start.secs + 1
        concat_nano = self.appended_msg.trajectory[0].joint_trajectory.points[-1].time_from_start.nsecs + 10
        for val in trajectories[plan].trajectory[0].joint_trajectory.points:
            val.time_from_start.secs += concat_seconds
            val.time_from_start.nsecs += concat_nano
            # totalnsecs = val.time_from_start.nsecs + concat_nano
            # if(totalnsecs > 1e9):
            #     val.time_from_start.nsecs += 1e9 - totalnsecs
            #     val.time_from_start.secs += 1


        self.appended_msg.trajectory[0].joint_trajectory.points += trajectories[plan].trajectory[0].joint_trajectory.points

    self.display_trajectory_publisher.publish(self.appended_msg)
    self.display_trajectory_pub.publish(self.appended_msg)

  def setPoseTargets(self):
    arm_group = self.arm_group
    self.arm_group.set_max_velocity_scaling_factor(1)

    currentPose = arm_group.get_current_pose()
    currentPose.pose.position.x = currentPose.pose.position.x + 0.1
    pose = Pose()

    Orientation = currentPose.pose.orientation

    newPose = PoseStamped()
    newPose.header = currentPose.header
    newPose.pose.orientation = Orientation

    arm_group.set_pose_target(currentPose)
    arm_group.plan()
    arm_group.go()

  def throw(self):
    arm_group = self.arm_group
    self.arm_group.set_max_velocity_scaling_factor(0.6)

    joint_positions = arm_group.get_current_joint_values()

    joint_positions[0] = joint_positions[0]
    joint_positions[1] = joint_positions[1]
    joint_positions[2] = joint_positions[2]
    joint_positions[3] = joint_positions[3]
    joint_positions[4] = joint_positions[4] 
    joint_positions[5] = -2.58

    arm_group.set_joint_value_target(joint_positions)

    try:
      # Plan the new trajectory
      self.main_plan = arm_group.plan()
      # print(arm_group.plan())
    except:
      rospy.logerr("Failed to plan trajectory.")
      # Call function to reset the position of target pose object to end effector location

    else:
      rospy.loginfo("Planned Initial Pose!")
      
  def execute_action_callback(self, data):  
    self.arm_group.set_max_velocity_scaling_factor(1)

    for segment in self.plan_array:  
        self.arm_group.execute(segment.trajectory[0], wait=True)    

    self.example_home_the_robot()


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
            time.sleep(3.0)


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
      # print(arm_group.plan())
    except:
      rospy.logerr("Failed to plan trajectory.")
      # Call function to reset the position of target pose object to end effector location

    else:
      rospy.loginfo("Planned Initial Pose!")
      arm_group.go(wait=True)

  def reach_pour_joint_values(self):
    
    arm_group = self.arm_group
    joint_positions = arm_group.get_current_joint_values()

    joint_positions[5] = -2.59

    arm_group.set_joint_value_target(joint_positions)

    try:
        # Plan the new trajectory
        self.main_plan = arm_group.plan()
        # arm_group.go(wait=True)

    except:
        rospy.logerr("Failed to plan trajectory.")
        # Call function to reset the position of target pose object to end effector location

    else:
        rospy.loginfo("Planned Initial Pose!")
        #   return self.wait_for_action_end_or_abort()

  def reach_pourhome_joint_values(self):
      arm_group = self.arm_group
      joint_positions = arm_group.get_current_joint_values()

      joint_positions[0] = -0.11
      joint_positions[1] = -0.67
      joint_positions[2] = 1.62
      joint_positions[3] = 0.27
      joint_positions[4] = -1.26
      joint_positions[5] = -0.76

      arm_group.set_joint_value_target(joint_positions)

      try:
        # Plan the new trajectory
        self.main_plan = arm_group.plan()
        arm_group.go(wait=True)

      except:
        rospy.logerr("Failed to plan trajectory.")
        # Call function to reset the position of target pose object to end effector location

      else:
        rospy.loginfo("Planned Initial Pose!")
      #   return self.wait_for_action_end_or_abort()

  def reach_home_joint_values(self):
    arm_group = self.arm_group
    joint_positions = arm_group.get_current_joint_values()

    joint_positions[0] = -0
    joint_positions[1] = -0.28
    joint_positions[2] = 1.311
    joint_positions[3] = 0
    joint_positions[4] = -1.047
    joint_positions[5] = 0

    arm_group.set_joint_value_target(joint_positions)

    try:
      # Plan the new trajectory
      self.main_plan = arm_group.plan()
      arm_group.go(wait=True)

    except:
      rospy.logerr("Failed to plan trajectory.")
      # Call function to reset the position of target pose object to end effector location

    else:
      rospy.loginfo("Planned Initial Pose!")
    #   return self.wait_for_action_end_or_abort()

  def get_current_joint_values(self):
    arm_group = self.arm_group
    joint_positions = arm_group.get_current_joint_values()
    for p in joint_positions: rospy.loginfo(p)
    return joint_positions


  def target_pose_callback(self, data):
    rospy.loginfo(data.pose.position)
    self.reach_cartesian_pose(pose=data.pose, tolerance=0.01, constraints=None)


  def FillCartesianWaypoint(self, new_x, new_y, new_z, new_theta_x, new_theta_y, new_theta_z, blending_radius):
    waypoint = Waypoint()
    cartesianWaypoint = CartesianWaypoint()

    cartesianWaypoint.pose.x = new_x
    cartesianWaypoint.pose.y = new_y
    cartesianWaypoint.pose.z = new_z
    cartesianWaypoint.pose.theta_x = new_theta_x
    cartesianWaypoint.pose.theta_y = new_theta_y
    cartesianWaypoint.pose.theta_z = new_theta_z
    cartesianWaypoint.reference_frame = CartesianReferenceFrame.CARTESIAN_REFERENCE_FRAME_BASE
    cartesianWaypoint.blending_radius = blending_radius
    waypoint.oneof_type_of_waypoint.cartesian_waypoint.append(cartesianWaypoint)

    return waypoint

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

  def move_with_pose(self):
    current_pose = self.get_cartesian_pose()

    new_pose_goal = current_pose
    new_pose_goal.position.x = current_pose.position.x + 0.1

    self.reach_cartesian_pose(pose=new_pose_goal, tolerance=0.01, constraints=None)


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
      # print(arm_group.plan())
    except:
      rospy.logerr("Failed to plan trajectory.")
      # Call function to reset the position of target pose object to end effector location

    else:
      # return self.wait_for_action_end_or_abort()
      pass      

    self.arm_group = arm_group
    # Plan

    rospy.loginfo("Planning")

    # return arm_group.go(wait=True)

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

  def move_arm(self, value):
    self.last_action_notif_type = None
    # Get the actual cartesian pose to increment it
    # You can create a subscriber to listen to the base_feedback
    # Here we only need the latest message in the topic though
    feedback = rospy.wait_for_message("/" + self.robot_name + "/base_feedback", BaseCyclic_Feedback)

    # Possible to execute waypointList via execute_action service or use execute_waypoint_trajectory service directly
    req = ExecuteActionRequest()
    trajectory = WaypointList()

    trajectory.waypoints.append(
        self.FillCartesianWaypoint(
            feedback.base.commanded_tool_pose_x,
            feedback.base.commanded_tool_pose_y,
            feedback.base.commanded_tool_pose_z,
            feedback.base.commanded_tool_pose_theta_x,
            feedback.base.commanded_tool_pose_theta_y,
            feedback.base.commanded_tool_pose_theta_z ,
            0)
    )

    trajectory.duration = 0
    trajectory.use_optimal_blending = False

    req.input.oneof_action_parameters.execute_waypoint_list.append(trajectory)

    # # Call the service
    rospy.loginfo("Sending the robot to the cartesian pose...")
    try:
        self.execute_action(req)
    except rospy.ServiceException:
        rospy.logerr("Failed to call ExecuteWaypointTrajectory")
        return False
    else:
        time.sleep(2)
        # return self.wait_for_action_end_or_abort()
  

if __name__ == '__main__':
  example = ExampleMoveItTrajectories()

