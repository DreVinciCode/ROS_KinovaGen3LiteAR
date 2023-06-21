#!/usr/bin/python3.7

import rospy
from std_msgs.msg import *
from kivy.uix.screenmanager import Screen
from kivymd.app import MDApp
from kivy.lang import Builder
from kivymd.uix.button import MDRectangleFlatButton

# import Pouring

class Pouring_Control_Panel(MDApp):
	
	def __init_(self, **kwargs):
		super().__init__(**kwargs)

	
	def build(self):
		self.screen=Builder.load_file('ros_gui.kv')
		self.screen.ids.Max_Velocity_value_text.text = str(round(rospy.get_param("/KinovaAR/MaxVelocity"),2))
		self.screen.ids.Tile_angle_value_text.text = str(round(rospy.get_param("/KinovaAR/TiltAngle"),2))
		return self.screen

	def horizontal_control_positive(self, *args):
		print("hor_pos")
		# execute_action_pub.publish(Empty())
		translate_pos_x_pub.publish(Empty())

	def horizontal_control_negative(self, *args):
		print("hor_neg")
		# execute_action_pub.publish(Empty())
		translate_neg_x_pub.publish(Empty())

	def vertical_control_positive(self, *args):
		print("vert_pos")
		# execute_action_pub.publish(Empty())
		translate_pos_z_pub.publish(Empty())

	def vertical_control_negative(self, *args):
		print("vert_neg")
		# execute_action_pub.publish(Empty())
		translate_neg_z_pub.publish(Empty())

	def tilt_angle_control_positive(self, *args):
		print("tilt_pos")
		self.screen.ids.Tile_angle_value_text.text = str(round(rospy.get_param("/KinovaAR/TiltAngle"),2))
		tilt_pos_pub.publish(Empty())

	def tilt_angle_control_negative(self, *args):
		print("tilt_neg")
		self.screen.ids.Tile_angle_value_text.text = str(round(rospy.get_param("/KinovaAR/TiltAngle"),2))
		tilt_neg_pub.publish(Empty())

	def max_vel_control_positive(self, *args):
		print("vel_pos")
		self.screen.ids.Max_Velocity_value_text.text = str(round(rospy.get_param("/KinovaAR/MaxVelocity"),2))
		velocity_pos_pub.publish(Empty())

	def max_vel_control_negative(self, *args):
		print("vel_neg")
		self.screen.ids.Max_Velocity_value_text.text = str(round(rospy.get_param("/KinovaAR/MaxVelocity"),2))
		velocity_neg_pub.publish(Empty())

	def execute_pour_action(self, *args):
		execute_action_pub.publish(Empty())


if __name__ == "__main__":
	
	execute_action_pub = rospy.Publisher("/KinovaAR/execute_action", Empty, queue_size=1)

	translate_pos_x_pub = rospy.Publisher("/KinovaAR/translatePositiveX", Empty, queue_size=1)
	translate_neg_x_pub = rospy.Publisher("/KinovaAR/translateNegativeX", Empty, queue_size=1)

	translate_pos_z_pub = rospy.Publisher("/KinovaAR/translatePositiveZ", Empty, queue_size=1)
	translate_neg_z_pub = rospy.Publisher("/KinovaAR/translateNegativeZ", Empty, queue_size=1)

	tilt_pos_pub = rospy.Publisher("/KinovaAR/tiltPositive", Empty, queue_size=1)
	tilt_neg_pub = rospy.Publisher("/KinovaAR/tiltNegative", Empty, queue_size=1)

	velocity_pos_pub = rospy.Publisher("/KinovaAR/MaxVelocity_inc", Empty, queue_size=1)
	velocity_neg_pub = rospy.Publisher("/KinovaAR/MaxVelocity_dec", Empty, queue_size=1)

	# pub = rospy.Publisher('/KinovaAR/MaxVelocity', Float32, queue_size=1)
	rospy.init_node('kivymd_gui', anonymous=True)	
	Pouring_Control_Panel().run()

