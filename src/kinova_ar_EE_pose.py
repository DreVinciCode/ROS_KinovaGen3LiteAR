#!/usr/bin/env python

from geometry_msgs.msg import Twist
from kortex_driver.msg import BaseCyclic_Feedback

import rospy

class messageConverter:

    def __init__(self):
    	rospy.init_node('PoseToPoint', anonymous = True)
    	self.base_feedback_sub = rospy.Subscriber("/my_gen3_lite/base_feedback", BaseCyclic_Feedback, self.base_feedback_callback)
        self.base_feedback_boundary_sub = rospy.Subscriber("/my_gen3_lite/base_feedback", BaseCyclic_Feedback, self.base_feedback_boundary_callback)
    	# self.Twist_pub = rospy.Publisher("/my_gen3_lite/end_effector_pose", Twist, queue_size=1)
    	self.Kinova_Position = rospy.Publisher("/my_gen3_lite/end_effector_pose", Twist, queue_size=1)
        self.Kinova_Velocity = rospy.Publisher("KinovaAR/end_effector_velocity", Twist, queue_size=1)

        self.KinovaAR_velocity_correction = rospy.Publisher("/my_gen3_lite/in/cartesian_velocity_mod", Twist, queue_size=1)

    	rospy.spin()

    def base_feedback_callback(self, data):
    	position_x = data.base.commanded_tool_pose_x
        position_y = data.base.commanded_tool_pose_y
        position_z = data.base.commanded_tool_pose_z

        angularX = data.base.commanded_tool_pose_theta_x
        angularY = data.base.commanded_tool_pose_theta_y
        angularZ = data.base.commanded_tool_pose_theta_z

        velocity_x = data.base.tool_twist_linear_x
        velocity_y = data.base.tool_twist_linear_y
        velocity_z = data.base.tool_twist_linear_z

        Twist_Message = Twist()
        Twist_Message.linear.x = velocity_x
        Twist_Message.linear.y = velocity_y
        Twist_Message.linear.z = velocity_z

        self.Kinova_Velocity.publish(Twist_Message)

    	Position_Message = Twist()
    	Position_Message.linear.x = position_x
    	Position_Message.linear.y = position_y
    	Position_Message.linear.z = position_z
        Position_Message.angular.x = angularX
        Position_Message.angular.y = angularY
        Position_Message.angular.z = angularZ

    	self.Kinova_Position.publish(Position_Message)


    # Ensure robotic end effector remains within the box boundary
    def base_feedback_boundary_callback(self, data):
        z_pos = data.base.commanded_tool_pose_z

        if(z_pos < rospy.get_param("/kinovaAR/z_min")):
            # What command to send? Stop command? Opposite velocity commands?
            
            # Trigger only if this condition is met. 
            Twist_Message = Twist()
            Twist_Message.linear.z = 0.1
            self.KinovaAR_velocity_correction.publish(Twist_Message)
            Twist_Message.linear.z = 0.0
            self.KinovaAR_velocity_correction.publish(Twist_Message)

        if(z_pos > rospy.get_param("/kinovaAR/z_max")):
            # What command to send? Stop command? Opposite velocity commands?
            
            # Trigger only if this condition is met. 
            Twist_Message = Twist()
            Twist_Message.linear.z = -0.1
            self.KinovaAR_velocity_correction.publish(Twist_Message)
            Twist_Message.linear.z = 0.0
            self.KinovaAR_velocity_correction.publish(Twist_Message)


if __name__ == '__main__':
    converter = messageConverter()
