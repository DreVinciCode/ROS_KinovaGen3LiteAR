#!/usr/bin/python3.7

import rospy
from std_msgs.msg import *
# from kivy.uix.screenmanager import Screen
from kivymd.app import MDApp
from kivy.lang import Builder

from kivymd.uix.button import MDRectangleFlatButton

class Pouring_Control_Panel(MDApp):
	
	def __init_(self, **kwargs):
		super().__init__(**kwargs)



	def build(self):
		self.screen=Builder.load_file('ros_gui.kv')
		self.screen.ids.Max_Velocity_value_text.text = "Fast"
		self.screen.ids.Tile_angle_value_text.text = "--"
		self.screen.ids.Horizontal_Position.text = str(rospy.get_param("/KinovaAR/Horizontal"))
		self.screen.ids.Vertical_Position.text = str(rospy.get_param("/KinovaAR/Vertical"))
		
		return self.screen

	def horizontal_control_positive(self, *args):
		translate_pos_x_pub.publish(Empty())
		value = rospy.get_param("/KinovaAR/Horizontal") + 1
		self.screen.ids.Horizontal_Position.text = str(value)

	def horizontal_control_negative(self, *args):
		translate_neg_x_pub.publish(Empty())
		value = rospy.get_param("/KinovaAR/Horizontal") - 1
		self.screen.ids.Horizontal_Position.text = str(value)

	def vertical_control_positive(self, *args):
		translate_pos_z_pub.publish(Empty())
		value = rospy.get_param("/KinovaAR/Vertical") + 1
		self.screen.ids.Vertical_Position.text = str(value)

	def vertical_control_negative(self, *args):
		translate_neg_z_pub.publish(Empty())
		value = rospy.get_param("/KinovaAR/Vertical") - 1
		self.screen.ids.Vertical_Position.text = str(value)

	def tilt_angle_control_positive(self, *args):
		self.screen.ids.Horizontal_Position.text = "test"
		tilt_pos_pub.publish(Empty())

	def tilt_angle_control_negative(self, *args):
		self.screen.ids.Tile_angle_value_text.text = str(round(rospy.get_param("/KinovaAR/TiltAngle"),2))
		tilt_neg_pub.publish(Empty())

	def set_tilt_angle(self, data):
		self.screen.ids.Tile_angle_value_text.text = str(data[1])
		tilt_set_pub.publish(data[0])

	def max_vel_control_positive(self, *args):
		self.screen.ids.Max_Velocity_value_text.text = str(round(rospy.get_param("/KinovaAR/MaxVelocity") * 100 ,2)) + "%"
		velocity_pos_pub.publish(Empty())

	def max_vel_control_negative(self, *args):
		self.screen.ids.Max_Velocity_value_text.text = str(round(rospy.get_param("/KinovaAR/MaxVelocity") * 100 ,2)) + "%"
		velocity_neg_pub.publish(Empty())

	def set_max_velocity(self, data):
		self.screen.ids.Max_Velocity_value_text.text = str(data[1])
		velocity_set_pub.publish(data[0])


	def execute_pour_action(self, *args):
		execute_action_pub.publish(Empty())

	def execute_reset_pour_position(self, *args):
		execute_reset_pour_pub.publish(Empty())

	def execute_reset_home_position(self, *args):
		self.screen.ids.Max_Velocity_value_text.text = "--"
		self.screen.ids.Tile_angle_value_text.text = "--"
		self.screen.ids.Horizontal_Position.text = str(0)
		self.screen.ids.Vertical_Position.text = str(0)
		execute_reset_position_pub.publish(Empty())

if __name__ == "__main__":
	
	execute_action_pub = rospy.Publisher("/KinovaAR/execute_action", Empty, queue_size=1)
	execute_reset_position_pub = rospy.Publisher("/KinovaAR/reset_pour_posiiton", Empty, queue_size=1)
	execute_reset_pour_pub = rospy.Publisher("/KinovaAR/reset_joint_posiiton", Empty, queue_size=1)

	translate_pos_x_pub = rospy.Publisher("/KinovaAR/translatePositiveX", Empty, queue_size=1)
	translate_neg_x_pub = rospy.Publisher("/KinovaAR/translateNegativeX", Empty, queue_size=1)

	translate_pos_z_pub = rospy.Publisher("/KinovaAR/translatePositiveZ", Empty, queue_size=1)
	translate_neg_z_pub = rospy.Publisher("/KinovaAR/translateNegativeZ", Empty, queue_size=1)

	tilt_pos_pub = rospy.Publisher("/KinovaAR/tiltPositive", Empty, queue_size=1)
	tilt_neg_pub = rospy.Publisher("/KinovaAR/tiltNegative", Empty, queue_size=1)
	tilt_set_pub = rospy.Publisher("/KinovaAR/setTiltValue", Float32, queue_size=1)

	velocity_pos_pub = rospy.Publisher("/KinovaAR/MaxVelocity_inc", Empty, queue_size=1)
	velocity_neg_pub = rospy.Publisher("/KinovaAR/MaxVelocity_dec", Empty, queue_size=1)
	velocity_set_pub = rospy.Publisher("/KinovaAR/setMaxVelocity", Float32, queue_size=1)

	rospy.init_node('kivymd_gui', anonymous=True)	
	Pouring_Control_Panel().run()


