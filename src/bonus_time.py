#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32
from kinova_ar.srv import SendStatistics, SendStatisticsResponse
from std_srvs.srv import Empty as EmptyService

class BonusTime(object):
    def __init__(self):

        rospy.init_node('bonus_time_manager', anonymous=True)

        self.pub = rospy.Subscriber("/KinovaAR/VisualCondition", Int32, self.update_visualization)

        rospy.Service('/KinovaAR/start_bonus_time', EmptyService, self.start)
        rospy.Service('/KinovaAR/stop_bonus_time', SendStatistics, self.send_data)

        self.start_time = None

        self.visualization_times = None

        self.current_visualization = None

        rospy.spin()

    def start(self, data) -> EmptyService:
        self.visualization_times = [0.0 for _ in range(data.number_of_visualizations)]

        self.start_time = rospy.Time.now()

        return EmptyService()

    def update_visualization(self, data):
        self.visualization_times[self.current_visualization] += (rospy.Time.now() - self.start_time).to_sec()
        self.current_visualization = data.data
        self.start_time = rospy.Time.now()

    def send_data(self, _ : SendStatistics) -> SendStatisticsResponse:
        self.visualization_times[self.current_visualization] += (rospy.Time.now() - self.start_time).to_sec()

        return self.visualization_times


if __name__ == '__main__':
    try:
        BonusTime()
    except rospy.ROSInterruptException:
        pass