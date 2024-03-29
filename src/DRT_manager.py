#!/usr/bin/env python3

'''
Author: Andre Cleaver
Modifed: BMK

This node is responsible for scheduling DRT triggers for the hololens
and sending accuracy metrics through a service to the study controller
'''

import time
import threading
import random

import rospy

from std_msgs.msg import Empty
from kinova_ar.srv import StopDRT, StopDRTResponse
from std_srvs.srv import Empty as EmptyService, EmptyResponse

class DRTManager():
    '''
    DRT_Manager class includes function that act as a middle man
    enabling communication with study controller and hololens
    '''

    def __init__(self):

        rospy.init_node('drt_manager', anonymous=True)

        self.pub = rospy.Publisher('/KinovaAR/DRT_signal', Empty, queue_size=1)

        rospy.Service('/KinovaAR/start_drt', EmptyService, self.start_drt)
        rospy.Service('/KinovaAR/stop_drt', StopDRT, self.send_score)

        self.run_drt = None
        self.counter = None
        self.success_counter = None
        self.signal_thread = None

    def start_drt(self, _) -> threading.Thread:
        '''
        start_drt callback starts timer and spawns thread
        that publishes to hololens
        '''
        self.success_counter = 0
        self.counter = 0

        self.run_drt = True

        self.signal_thread = threading.Thread(target=self.drt_signal_thread)
        self.signal_thread.daemon = True
        self.signal_thread.start()

        return EmptyResponse()

    def drt_signal_thread(self):
        '''
        publishes triggers to the hololens 
        and records number of triggers sent
        '''
        time_sleep = random.uniform(2.5, 5)
        while self.run_drt:
            time.sleep(time_sleep)
            self.pub.publish(Empty())
            self.counter += 1
            rospy.loginfo("Published message. Total count: %d", self.counter)

            try:
                send_time = rospy.Time.now()

                rospy.wait_for_message('/KinovaAR/DRT_response', Empty, timeout=rospy.Duration(2.5))
                time_taken = (rospy.Time.now() - send_time).to_sec()

                rospy.loginfo("Recieved response from participant")
                self.success_counter += 1
                time_sleep = random.uniform(3 - time_taken, 5 - time_taken)
            except rospy.ROSException:
                rospy.logerr("No response recieved...")
                time_sleep = random.uniform(0.5, 2.5)

            rospy.loginfo(f"Current score: {self.success_counter / self.counter}")

    def send_score(self, _ : StopDRT) -> StopDRTResponse:
        '''return drt accuracy score to service sender
        '''

        self.run_drt = False

        if self.signal_thread is None:
            rospy.logerr("No thread is running....")
            return

        self.signal_thread.join()
        self.signal_thread = None

        score = self.success_counter / self.counter
        rospy.loginfo(f"Final score: {score}")

        return StopDRTResponse(score)


if __name__ == '__main__':
    try:
        DRT = DRTManager()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
