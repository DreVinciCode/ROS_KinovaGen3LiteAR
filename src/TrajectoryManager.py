#!/usr/bin/env python

from moveit_msgs.msg import *
from std_msgs.msg import *
from kinova_study.msg import LoadTrajectory, TrajectoryInfo, LoadTrajectoryMultiArray
import rospy

class KinovaARTrajectoryManager(object):
    
    def __init__(self):
        
        self.record_check = False

        self.plan1_check = False
        self.plan2_check = False

        self.FirstTrajectory = DisplayTrajectory()
        self.SecondTrajectory = DisplayTrajectory()

        self.trajectories_msg = LoadTrajectoryMultiArray()
        self._planA = LoadTrajectory()
        self._planB = LoadTrajectory()

        self.received_messages = []

        self.trajectoryInfo = TrajectoryInfo()

        try:
            rospy.init_node('TrajectoryManager')
            self.trajectory_planner_sub = rospy.Subscriber("/my_gen3_lite/move_group/display_planned_path", DisplayTrajectory, self.trajectory_planner_callback)

            self.FirstTrajectory_Publisher = rospy.Publisher("/KinovaAR/FirstTrajectory", DisplayTrajectory, queue_size=1)
            self.SecondTrajectory_Publisher = rospy.Publisher("/KinovaAR/SecondTrajectory", DisplayTrajectory, queue_size=1)

            self.FirstTrajectoryDisplay_Publisher = rospy.Publisher("/KinovaAR/FirstTrajectoryDisplay", DisplayTrajectory, queue_size=1)
            self.SecondTrajectoryDisplay_Publisher = rospy.Publisher("/KinovaAR/SecondTrajectoryDisplay", DisplayTrajectory, queue_size=1)

            self.KeyPress_sub = rospy.Subscriber("/KinovaAR/save", Empty, self.change_bool_callback)

            self.TrajectorySubscriber_sub = rospy.Subscriber("/KinovaAR/TrajectoryCombined", LoadTrajectory, self.temp)
            self.PublishTrajectories_msg = rospy.Publisher("/KinovaAR/TrajectoryCombined_test", TrajectoryInfo, queue_size=1)


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


    def temp(self, data):

        # self.grasp_regions = {"middle-body": 0, "lower-body": 1, "upper-handle": 2, "lower-handle": 3, "middle-rim": 4, "lower-rim": 5, "rim-body": 6}
        self.received_messages.append(data)

        if len(self.received_messages) == 2:

            self._planA = self.received_messages[0]
            self._planB = self.received_messages[1]


            self.trajectories_msg.trajectories =  self.received_messages

            self.trajectoryInfo.trajectories = [self._planA, self._planB]
            self.trajectoryInfo.grasp_region = [0, 0]

            self.PublishTrajectories_msg.publish(self.trajectoryInfo)
            print("2 recorded")

    def change_bool_callback(self, data):
        
        self.record_check = not(self.record_check)   
        print("Recording: " + str(self.record_check))


    def trajectory_planner_callback(self, data): 

        h = std_msgs.msg.Header()
        h.stamp = rospy.Time.now() 

        data.trajectory[0].joint_trajectory.header = h

        # if(self.record_check):
        if(True):
            self.FirstTrajectory = data
            self.FirstTrajectory_Publisher.publish(self.FirstTrajectory)
            # self.SecondTrajectoryDisplay_Publisher.publish(data)
            self.FirstTrajectoryDisplay_Publisher.publish(self.FirstTrajectory)
        else:
            pass


if __name__ == '__main__':
    trajectory = KinovaARTrajectoryManager()
