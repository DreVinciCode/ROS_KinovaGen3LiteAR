#!/usr/bin/env python

from geometry_msgs.msg import Twist
from kortex_driver.msg import BaseCyclic_Feedback

import rospy

class messageConverter:

    def __init__(self):
    	rospy.init_node('PoseToPoint', anonymous = True)
    	self.base_feedback_sub = rospy.Subscriber("/my_gen3_lite/base_feedback", BaseCyclic_Feedback, self.base_feedback_callback)
    	self.Twist_pub = rospy.Publisher("/my_gen3_lite/end_effector_pose", Twist, queue_size=1)
    	rospy.spin()

    def base_feedback_callback(self, data):
    	linearX = data.base.commanded_tool_pose_x
        linearY = data.base.commanded_tool_pose_y
        linearZ = data.base.commanded_tool_pose_z

        angularX = data.base.commanded_tool_pose_theta_x
        angularY = data.base.commanded_tool_pose_theta_y
        angularZ = data.base.commanded_tool_pose_theta_z

    	Twist_Message = Twist()
    	Twist_Message.linear.x = linearX
    	Twist_Message.linear.y = linearY
    	Twist_Message.linear.z = linearZ
        Twist_Message.angular.x = angularX
        Twist_Message.angular.y = angularY
        Twist_Message.angular.z = angularZ

    	self.Twist_pub.publish(Twist_Message)

if __name__ == '__main__':
    converter = messageConverter()