#!/usr/bin/python3.7

import rospy
from std_msgs.msg import *
from kivy.uix.screenmanager import Screen
from kivymd.app import MDApp
from kivy.lang import Builder
from kivymd.uix.button import MDRectangleFlatButton

class ARViz_GUI(MDApp):
	
	def __init_(self, **kwargs):
		super().__init__(**kwargs)
	
	def build(self):
		self.screen=Builder.load_file('ros_gui.kv')
		return self.screen

	def horizontal_control_positive(self, *args):
		# Pouring.test_function()
		print(str(*args))
		# execute_action_pub.publish(Empty())
		translate_pos_x_pub.publish(Empty())

	def horizontal_control_negative(self, *args):
		# Pouring.test_function()
		print(str(*args))
		# execute_action_pub.publish(Empty())
		translate_neg_x_pub.publish(Empty())

	def vertical_control_positive(self, *args):
		# Pouring.test_function()
		print(str(*args))
		# execute_action_pub.publish(Empty())
		translate_pos_x_pub.publish(Empty())

	def vertical_control_negative(self, *args):
		# Pouring.test_function()
		print(str(*args))
		# execute_action_pub.publish(Empty())
		translate_neg_x_pub.publish(Empty())

	def tilt_angle_control_positive(self, *args):
		# Pouring.test_function()
		print(str(*args))
		# execute_action_pub.publish(Empty())
		tilt_pos_pub.publish(Empty())

	def tilt_angle_control_negative(self, *args):
		# Pouring.test_function()
		print(str(*args))
		# execute_action_pub.publish(Empty())
		tilt_neg_pub.publish(Empty())

if __name__ == "__main__":
	
	execute_action_pub = rospy.Publisher("/KinovaAR/execute_action", Empty, queue_size=1)

	translate_pos_x_pub = rospy.Publisher("/KinovaAR/translatePositiveX", Empty, queue_size=1)
	translate_neg_x_pub = rospy.Subscriber("/KinovaAR/translateNegativeX", Empty, queue_size=1)

	translate_pos_z_pub = rospy.Publisher("/KinovaAR/translatePositiveZ", Empty, queue_size=1)
	translate_neg_z_pub = rospy.Subscriber("/KinovaAR/translateNegativeZ", Empty, queue_size=1)

	tilt_pos_pub = rospy.Publisher("/KinovaAR/tiltPositive", Empty, queue_size=1)
	tilt_neg_pub = rospy.Subscriber("/KinovaAR/tiltNegative", Empty, queue_size=1)

	# pub = rospy.Publisher('/KinovaAR/MaxVelocity', Float32, queue_size=1)
	rospy.init_node('kivymd_gui', anonymous=True)	
	ARViz_GUI().run()

