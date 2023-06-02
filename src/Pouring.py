#!/usr/bin/env python

from geometry_msgs.msg import *
from kortex_driver.srv import *
from kortex_driver.msg import *
from moveit_msgs.msg import *
from kinova_study.msg import load_trajectory
from sensor_msgs.msg import JointState

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
    
    self.max_velocity = 1
    
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

  def max_velocity_change_callback(self, data):
    self.max_velocity = data.data
    self.plan_pour_speed()

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
    joint_positions[5] = -0.453

    arm_group.set_joint_value_target(joint_positions)

    try:
      # Plan the new trajectory
      self.main_plan = arm_group.plan()
      # plan = DisplayTrajectory()
      # plan.trajectory[0].joint_trajectory = self.main_plan.joint_trajectory


      # self.display_trajectory_pub.publish(plan)
      # # print(arm_group.plan())
    except:
      rospy.logerr("Failed to plan trajectory.")
      # Call function to reset the position of target pose object to end effector location

    else:
      rospy.loginfo("Planned Initial Pose!")
      
  def execute_action_callback(self, data):  

    self.arm_group.go(wait=True)

    # for segment in self.plan_array:  
    #     self.arm_group.execute(segment.trajectory[0], wait=True)    

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

  def reach_pour_home_joint_values(self):
      arm_group = self.arm_group
      joint_positions = arm_group.get_current_joint_values()

      joint_positions[0] = -0.0346
      joint_positions[1] = -0.829
      joint_positions[2] = 1.843
      joint_positions[3] = 1.539
      joint_positions[4] = -1.151
      joint_positions[5] = 1.648

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


  def move_with_pose(self):
    current_pose = self.get_cartesian_pose()

    new_pose_goal = current_pose
    new_pose_goal.position.x = current_pose.position.x + 0.1

    self.reach_cartesian_pose(pose=new_pose_goal, tolerance=0.01, constraints=None)


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

