#!/usr/bin/env python

from moveit_msgs.msg import *
from std_msgs.msg import *
import rospy

class KinovaARTrajectoryManager(object):
    
    def __init__(self):
        
        self.record_check = False

        self.plan1_check = False
        self.plan2_check = False
        self.FirstTrajectory = DisplayTrajectory()
        self.SecondTrajectory = DisplayTrajectory()

        try:
            rospy.init_node('TrajectoryManager')
            self.trajectory_planner_sub = rospy.Subscriber("/my_gen3_lite/move_group/display_planned_path", DisplayTrajectory, self.trajectory_planner_callback)
            self.FirstTrajectory_Publisher = rospy.Publisher("/KinovaAR/FirstTrajectory", DisplayTrajectory, queue_size=1)
            self.SecondTrajectory_Publisher = rospy.Publisher("/KinovaAR/SecondTrajectory", DisplayTrajectory, queue_size=1)
            self.DisplayPlanner_sub = rospy.Subscriber("/KinovaAR/DisplayTrajectories", Empty, self.display_planners)

            self.KeyPress_sub = rospy.Subscriber("/KinovaAR/save", Empty, self.change_bool_callback)


        except Exception as e:   
            print (e)
            self.is_init_success = False
        else:
            self.is_init_success = True
            success = self.is_init_success

        try:
            rospy.spin()
        except:
            rospy.logerr("Failed to call ROS spin")

    def change_bool_callback(self, data):
        
        self.record_check = not(self.record_check)   
        print("Recording: " + str(self.record_check))


    def trajectory_planner_callback(self, data):     
        h = std_msgs.msg.Header()
        h.stamp = rospy.Time.now() 

        data.trajectory[0].joint_trajectory.header = h

        if(self.record_check):
            self.FirstTrajectory = data
            self.FirstTrajectory_Publisher.publish(self.FirstTrajectory)
        else:
            pass



        # if(self.plan1_check):
        #     self.FirstTrajectory = data
            
        #     rospy.loginfo("First Trajectory Recorded!")
        #     self.FirstTrajectory_Publisher.publish(self.FirstTrajectory)

        # if(self.plan2_check):
        #     self.SecondTrajectory = data
        #     self.plan2_check = False
        #     rospy.loginfo("Second Trajectory Recorded!")
        #     self.SecondTrajectory_Publisher.publish(self.SecondTrajectory)

        # if( not self.plan1_check and not self.plan2_check):
        #     self.plan1_check = True

        # elif(not self.plan2_check and self.plan1_check):
        #     self.plan2_check = True
        #     self.plan1_check = False

        # elif(self.plan2_check and not self.plan1_check):
        #     self.plan2_check = False
        #     self.plan1_check = False


    def display_planners(self, data):
        self.SecondTrajectory_Publisher.publish(self.SecondTrajectory)
        self.FirstTrajectory_Publisher.publish(self.FirstTrajectory)


if __name__ == '__main__':
    trajectory = KinovaARTrajectoryManager()
