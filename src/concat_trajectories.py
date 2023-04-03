#!/usr/bin/env python3

import rospy 
from moveit_msgs.msg import DisplayTrajectory

class concat_trajectories:
    def __init__(self):
        rospy.Subscriber("/KinovaAR/FirstTrajectory",
                            DisplayTrajectory, self.concat)
        
        rospy.Publisher("/KinovaAR/ConcatTrajectory", DisplayTrajectory, queue_size=1)

    def concat(self, data):
        second_msg = rospy.wait_for_message("/KinovaAR/FirstTrajectory", DisplayTrajectory)
        
        appended_msg = DisplayTrajectory()
        appended_msg.trajectory = data.trajectory + second_msg.trajectory
        appended_msg.RobotState = data.RobotState

        self.concat_publisher.publish(appended_msg)