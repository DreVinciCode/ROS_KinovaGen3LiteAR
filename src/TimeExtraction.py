#!/usr/bin/env python

import rospy

from moveit_msgs.msg import *

class TimeExtration:

    def __init__(self):
    	rospy.init_node('KinovaAR_TimeExtraction', anonymous = True)
        self.robot_name = rospy.get_param('~robot_name', "my_gen3_lite")

        # Subscribers to retrieve Planner Time and Execution Time
        self.plannerTime_sub = rospy.Subscriber('/' + self.robot_name + "/move_group/result", MoveGroupActionResult, self.plannerTimer_callback)
        self.trajectoryTime_sub = rospy.Subscriber('/' + self.robot_name + "/execute_trajectory/result", ExecuteTrajectoryActionResult, self.executeTimer_callback)


        rospy.spin()
        
    def plannerTimer_callback(self, data):
        plannerTime = data.result.planning_time
        # rospy.logerr(plannerTime)

    def executeTimer_callback(self, data):
        executeTime = (data.header.stamp.secs + data.header.stamp.nsecs * 1e-9) - (data.status.goal_id.stamp.secs + data.status.goal_id.stamp.nsecs * 1e-9)
        # rospy.logerr(executeTime)

if __name__ == '__main__':
    Time = TimeExtration()
