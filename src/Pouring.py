#!/usr/bin/env python

from geometry_msgs.msg import *
from kortex_driver.srv import *
from kortex_driver.msg import *
from moveit_msgs.msg import *
from kinova_study.msg import load_trajectory
from sensor_msgs.msg import JointState

# from master_gui import Pouring_Control_Panel

import sys
import time
import rospy
import moveit_commander
import math
import time
import numpy as np

from math import pi
from std_srvs.srv import Empty
from std_msgs.msg import *

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
    
    self.max_velocity = 1.0
    self.max_angle = 1.6
    self.horizontal_pos = 0
    self.vertical_pos = 0

    self.pause_time = 1

    rospy.set_param("/KinovaAR/Vertical", self.vertical_pos)
    rospy.set_param("/KinovaAR/Horizontal", self.horizontal_pos)
    rospy.set_param("/KinovaAR/MaxVelocity", self.max_velocity)
    rospy.set_param("/KinovaAR/TiltAngle", self.max_angle)

    
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

      self.max_velocity_change_sub = rospy.Subscriber("/KinovaAR/MaxVelocity", Float32, self.max_velocity_change_callback)

      self.max_angle_change_sub = rospy.Subscriber("/KinovaAR/AngleLimit", Float32, self.max_angle_change_callback)

      self.translate_pos_x_sub = rospy.Subscriber("/KinovaAR/translatePositiveX", Empty, self.translate_pos_x_callback)

      self.translate_neg_x_sub = rospy.Subscriber("/KinovaAR/translateNegativeX", Empty, self.translate_neg_x_callback)
      
      self.translate_pos_z_sub = rospy.Subscriber("/KinovaAR/translatePositiveZ", Empty, self.translate_pos_z_callback)
      
      self.translate_neg_z_sub = rospy.Subscriber("/KinovaAR/translateNegativeZ", Empty, self.translate_neg_z_callback)

      self.velocity_inc_pub = rospy.Subscriber("/KinovaAR/MaxVelocity_inc", Empty, self.set_max_vel_inc_callback)

      self.velocity_dec_pub = rospy.Subscriber("/KinovaAR/MaxVelocity_dec", Empty, self.set_max_vel_dec_callback)

      self.angle_tilt_inc_pub = rospy.Subscriber("/KinovaAR/tiltPositive", Empty, self.set_max_angle_inc_callback)

      self.angle_tilt_dec_pub = rospy.Subscriber("/KinovaAR/tiltNegative", Empty, self.set_max_angle_dec_callback)

      self.set_tilt_angle_sub = rospy.Subscriber("/KinovaAR/setTiltValue", Float32, self.set_max_tilt_callback)

      self.set_max_velocity_sub = rospy.Subscriber("/KinovaAR/setMaxVelocity", Float32, self.set_max_velocity_callback)

      self.reset_pour_position_pub = rospy.Subscriber("/KinovaAR/reset_pour_posiiton", Empty, self.resetToHome)
      # self.plan_pour_actions_sub = rospy.Subscriber("/KinovaAR/plan_pour", Empty, self.plan_pour_speed)

    except Exception as e:   
      print (e)
      self.is_init_success = False
    else:
      self.is_init_success = True

    success = self.is_init_success

    if success:   
        # self.get_current_joint_values() 
        self.reach_pour_home_joint_values()

    else:
       print("Failed")

    try:
      rospy.spin()
    except:
      rospy.logerr("Failed to call ROS spin")

  def set_max_tilt_callback(self, data):
    self.max_angle = data.data
    rospy.set_param("/KinovaAR/TiltAngle", self.max_angle)
    self.max_angle_change_callback(self.max_angle)

  def set_max_angle_dec_callback(self, data):
    self.dec_max_tilt()

  def set_max_angle_inc_callback(self, data):
    self.inc_max_tilt()

  def set_max_velocity_callback(self, data):
    self.max_velocity = data.data
    rospy.set_param("/KinovaAR/MaxVelocity", self.max_velocity)
    self.max_velocity_change_callback(self.max_velocity)
   
  def set_max_vel_inc_callback(self, data):
    self.inc_max_velocity()

  def set_max_vel_dec_callback(self, data):
    self.dec_max_velocity()

  def translate_pos_x_callback(self, data):
    self.translate_along_pos_x()
    time.sleep(self.pause_time)
    self.plan_pour_speed()

  def translate_neg_x_callback(self, data):
    self.translate_along_neg_x()
    time.sleep(self.pause_time)
    self.plan_pour_speed()

  def translate_pos_z_callback(self, data):
    self.translate_along_pos_z()
    time.sleep(self.pause_time)
    self.vertical_pos = self.vertical_pos + 1
    rospy.set_param("/KinovaAR/Vertical", self.vertical_pos)
    self.plan_pour_speed()

  def translate_neg_z_callback(self, data):
    self.translate_along_neg_z()
    time.sleep(self.pause_time)
    self.vertical_pos = self.vertical_pos - 1
    rospy.set_param("/KinovaAR/Vertical", self.vertical_pos)
    self.plan_pour_speed()

  def max_angle_change_callback(self, data):
    self.plan_pour_speed()

  def max_velocity_change_callback(self, data):
    self.plan_pour_speed()

  def plan_pour_speed(self):
    arm_group = self.arm_group
    arm_group.set_max_velocity_scaling_factor(self.max_velocity)
    # arm_group.set_max_acceleration_scaling_factor(0.01)

    arm_group.set_goal_position_tolerance(0.1)
    joint_positions = arm_group.get_current_joint_values()

    joint_positions[0] = joint_positions[0]
    joint_positions[1] = joint_positions[1]
    joint_positions[2] = joint_positions[2]
    joint_positions[3] = joint_positions[3]
    joint_positions[4] = joint_positions[4] 
    # joint_positions[5] = -0.453
    joint_positions[5] = self.max_angle

    arm_group.set_joint_value_target(joint_positions)

    try:
      self.main_plan = arm_group.plan()

    except:
      rospy.logerr("Failed to plan trajectory.")

    else:
      rospy.loginfo("Planned Initial Pose!")
      
  def execute_action_callback(self, data):  
    self.arm_group.go(wait=True)
    time.sleep(3)
    self.reach_pour_home_joint_values()
    self.arm_group.go(wait=True)
   
  def resetToHome(self, data):
    self.horizontal_pos = 0
    self.vertical_pos = 0
    self.max_velocity = 1
    self.max_angle = 1.6

    rospy.set_param("/KinovaAR/Horizontal", self.horizontal_pos)
    rospy.set_param("/KinovaAR/Vertical", self.vertical_pos)

    self.reach_pour_home_joint_values()
    self.arm_group.go(wait=True)

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

  def reach_pour_home_joint_values(self):
      arm_group = self.arm_group
      joint_positions = arm_group.get_current_joint_values()

      joint_positions[0] = -0.043
      joint_positions[1] = -0.739
      joint_positions[2] = 2.144
      joint_positions[3] = 1.532
      joint_positions[4] = -1.357
      joint_positions[5] = 1.6

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


  def translate_along_pos_x(self):
    current_pose = self.get_cartesian_pose()
    new_pose_goal = current_pose
    new_pose_goal.position.y = current_pose.position.y - 0.01
    self.reach_cartesian_pose(pose=new_pose_goal, tolerance=0.001, constraints=None)
    self.horizontal_pos = self.horizontal_pos + 1
    rospy.set_param("/KinovaAR/Horizontal", self.horizontal_pos)


  def translate_along_neg_x(self):
    current_pose = self.get_cartesian_pose()
    new_pose_goal = current_pose
    new_pose_goal.position.y = current_pose.position.y + 0.01
    self.reach_cartesian_pose(pose=new_pose_goal, tolerance=0.001, constraints=None)
    self.horizontal_pos = self.horizontal_pos - 1
    rospy.set_param("/KinovaAR/Horizontal", self.horizontal_pos)

  def translate_along_pos_z(self):
    current_pose = self.get_cartesian_pose()
    new_pose_goal = current_pose
    new_pose_goal.position.z = current_pose.position.z + 0.01
    self.reach_cartesian_pose(pose=new_pose_goal, tolerance=0.001, constraints=None)

  def translate_along_neg_z(self):
    current_pose = self.get_cartesian_pose()
    new_pose_goal = current_pose
    new_pose_goal.position.z = current_pose.position.z - 0.01
    self.reach_cartesian_pose(pose=new_pose_goal, tolerance=0.001, constraints=None)

  def dec_max_velocity(self):
    self.max_velocity = self.max_velocity - 0.05
    if(self.max_velocity < 0.05):
      self.max_velocity = 0.05

    rospy.set_param("/KinovaAR/MaxVelocity", self.max_velocity)
    self.max_velocity_change_callback(self.max_velocity)
    
  def inc_max_velocity(self):
    self.max_velocity = self.max_velocity + 0.05
    if(self.max_velocity > 1):
      self.max_velocity = 1.0

    rospy.set_param("/KinovaAR/MaxVelocity", self.max_velocity)
    self.max_velocity_change_callback(self.max_velocity)
    
  def inc_max_tilt(self):
    self.max_angle = self.max_angle + 0.05
    if(self.max_angle > 1.6):
      self.max_angle = 1.6

    rospy.set_param("/KinovaAR/TiltAngle", self.max_angle)
    self.max_angle_change_callback(self.max_angle)
    
  def dec_max_tilt(self):
    self.max_angle = self.max_angle - 0.05
    if(self.max_angle < -1.6):
      self.max_angle = -1.6
    
    rospy.set_param("/KinovaAR/TiltAngle", self.max_angle)
    self.max_angle_change_callback(self.max_angle)
    
  def get_cartesian_pose(self):
    arm_group = self.arm_group
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
      arm_group.go(wait=True)

    except:
      rospy.logerr("Failed to plan trajectory.")
      # Call function to reset the position of target pose object to end effector location

    else:
      # return self.wait_for_action_end_or_abort()
      pass      

    self.arm_group = arm_group

    rospy.loginfo("Planning")
  

  def source_configurations_callback(self, value):

    if(value == 0):
      self.max_angle = 0
      self.max_velocity = 1
    pass



if __name__ == '__main__':
  example = ExampleMoveItTrajectories()

