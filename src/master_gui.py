#!/usr/bin/python3.7

import rospy
from std_msgs.msg import *
from kivy.uix.screenmanager import Screen
from kivymd.app import MDApp
from kivy.lang import Builder
from kivymd.uix.button import MDRectangleFlatButton

# import Pouring


class ARViz_GUI(MDApp):
	
	def __init_(self, **kwargs):
		super().__init__(**kwargs)

	
	def build(self):
		self.screen=Builder.load_file('ros_gui.kv')
		return self.screen

	def inc_person_threshold(self, *args):
		# Pouring.test_function()
		print(str(*args))
		# execute_action_pub.publish(Empty())
		translate_pos_x_pub.publish(Empty())

	def dec_person_threshold(self, *args):
		# Pouring.test_function()
		print(str(*args))
		# execute_action_pub.publish(Empty())
		translate_pos_x_pub.publish(Empty())


if __name__ == "__main__":
	
	execute_action_pub = rospy.Publisher("/KinovaAR/execute_action", Empty, queue_size=1)

	translate_pos_x_pub = rospy.Publisher("/KinovaAR/translatePositiveX", Empty, queue_size=1)
	translate_neg_x_pub = rospy.Subscriber("/KinovaAR/translateNegativeX", Empty, queue_size=1)



	# pub = rospy.Publisher('/KinovaAR/MaxVelocity', Float32, queue_size=1)
	rospy.init_node('kivymd_gui', anonymous=True)	
	ARViz_GUI().run()

