#!/usr/bin/env python3

import keyboard
import rospy
import threading
import random
from std_msgs.msg import Empty, Float32
import time

class DRT_Manager(object):
    def __init__(self):

        rospy.init_node('drt_manager', anonymous=True)
        self.pub = rospy.Publisher('/KinovaAR/DRT_signal', Empty, queue_size=1)
        self.sub = rospy.Subscriber('/KinovaAR/DRT_response', Empty, self.response_callback)
        
        self.accuracy_pub = rospy.Publisher("/KinovaAR/DRT_accuracy", Float32, queue_size=1)

        self.response_received = False

        self.key_pressed = False
        self.counter = 0
        self.success_counter = 0
        self.next_publish_time = rospy.Time.now()

        signal_thread = threading.Thread(target=self.drt_signal_thread)
        signal_thread.daemon = True
        signal_thread.start()
        # while not rospy.is_shutdown():
        #     self.drt_signal_thread()

    def response_callback(self, data):
        current_time = rospy.Time.now()
        if (self.next_publish_time - current_time) >= rospy.Duration(2.5):
            rospy.loginfo("Response received within 2500ms after publishing")
            self.success_counter += 1
            # self.calculate_rate()
        else:
            rospy.loginfo("Response NOT received within 2500ms after publishing")
        self.response_received = True

    def calculate_rate(self):
        accuracy = self.success_counter / self.counter
        self.accuracy_pub.publish(accuracy)
        print(accuracy) 


    def drt_signal_thread(self):
        while not rospy.is_shutdown():
            current_time = rospy.Time.now()
            if current_time >= self.next_publish_time and not self.response_received:
                rand = random.uniform(3, 5)
                self.next_publish_time = current_time + rospy.Duration(rand)
                self.pub.publish(Empty())
                self.counter += 1
                print("Published message. Total count: %d", self.counter)

            self.response_received = False
            time.sleep(0.1)  # Adjust the sleep time as needed



if __name__ == '__main__':
    try:
        DRT = DRT_Manager()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass