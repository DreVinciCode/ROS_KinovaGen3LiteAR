#!/usr/bin/env python

from geometry_msgs.msg import Twist
from kortex_driver.msg import TwistCommand
from std_msgs.msg import Float32

import rospy

class BoundaryManager:

    def __init__(self):
    	rospy.init_node('KinovaAR_BoundaryManager', anonymous = True)

        self.Boundary_Bottom_sub = rospy.Subscriber("/kinovaAR/boundary/bottom", Float32, self.boundary_bottom)
        self.Boundary_Top_sub = rospy.Subscriber("/kinovaAR/boundary/top", Float32, self.boundary_top)
        
        rospy.spin()


    def boundary_bottom(self, data):
        rospy.set_param("/kinovaAR/z_min", data.data)

    def boundary_top(self, data):
        rospy.set_param("/kinovaAR/z_max", data.data)

if __name__ == '__main__':
    script = BoundaryManager()