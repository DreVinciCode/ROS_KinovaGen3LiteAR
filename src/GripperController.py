#!/usr/bin/env python
###
# KINOVA (R) KORTEX (TM)
#
# Copyright (c) 2019 Kinova inc. All rights reserved.
#
# This software may be modified and distributed 
# under the terms of the BSD 3-Clause license.
#
# Refer to the LICENSE file for details.
#
###

import sys
import rospy
import time

from kortex_driver.srv import *
from kortex_driver.msg import *

from std_msgs.msg import Float32, Header

class ExampleFullArmMovement:
    def __init__(self):
        
        rospy.init_node('example_full_arm_movement_python')


        # Get node params
        self.robot_name = rospy.get_param('~robot_name', "my_gen3_lite")
        self.degrees_of_freedom = rospy.get_param("/" + self.robot_name + "/degrees_of_freedom", 7)
        self.is_gripper_present = rospy.get_param("/" + self.robot_name + "/is_gripper_present", False)

        rospy.loginfo("Using robot_name " + self.robot_name + " , robot has " + str(self.degrees_of_freedom) + " degrees of freedom and is_gripper_present is " + str(self.is_gripper_present))

        send_gripper_command_full_name = '/' + self.robot_name + '/base/send_gripper_command'
        rospy.wait_for_service(send_gripper_command_full_name)
        self.send_gripper_command = rospy.ServiceProxy(send_gripper_command_full_name, SendGripperCommand)

        self.GripperSubscriber = rospy.Subscriber("/KinovaAR/gripper_cmd/goal_mod", Float32, self.Gripper_callback)

        rospy.spin()

    def Gripper_callback(self, data):
        gripper_pose = data.data

        if(gripper_pose >= 0 and gripper_pose <= 1):

            req = SendGripperCommandRequest()
            finger = Finger()
            finger.finger_identifier = 0
            finger.value = gripper_pose
            req.input.gripper.finger.append(finger)
            req.input.mode = GripperMode.GRIPPER_POSITION
            # rospy.loginfo(GripperMode.GRIPPER_POSITION)
            self.send_gripper_command(req)
            time.sleep(0.1)

if __name__ == "__main__":
    ex = ExampleFullArmMovement()

