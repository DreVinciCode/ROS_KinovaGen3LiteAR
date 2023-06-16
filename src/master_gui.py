#!/usr/bin/python3.7

import rospy
from std_msgs.msg import *
from kivy.uix.screenmanager import Screen
from kivymd.app import MDApp
from kivy.lang import Builder
from kivymd.uix.button import MDRectangleFlatButton

import Pouring


class ARViz_GUI(MDApp):
	
	def __init_(self, **kwargs):
		super().__init__(**kwargs)

	
	def build(self):
		self.screen=Builder.load_file('ros_gui.kv')
		return self.screen

	def inc_person_threshold(self, *args):
	    print(str(*args))
	


if __name__ == "__main__":
	
	pub = rospy.Publisher('/KinovaAR/MaxVelovity', Float32, queue_size=1)
	rospy.init_node('kivymd_gui', anonymous=True)	
	ARViz_GUI().run()

