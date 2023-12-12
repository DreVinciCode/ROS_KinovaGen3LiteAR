#!/usr/bin/env python3

import keyboard
import rospy
import time
from std_msgs.msg import Empty

class Recorder(object):
    def __init__(self):

        rospy.init_node('RecordKeypress')
        self.Empty_Publisher = rospy.Publisher("/KinovaAR/save", Empty, queue_size=1)
        self.DRT_Publisher = rospy.Publisher("/KinovaAR/DRT_response", Empty, queue_size=1)
        self.DRT_Subscriber = rospy.Subscriber("/KinovaAR/DRT_signal", Empty, self.signalUpdate)
        self.record_check = True
        self.key_pressed = False
        self.signalReceived = False

        while not rospy.is_shutdown():
            self.keyboard_listener()
        
    def signalUpdate(self, data):
        self.signalReceived = True


    def keyboard_listener(self):
        if keyboard.is_pressed('a') and not self.key_pressed and self.signalReceived:
            self.DRT_Publisher.publish(Empty())
            self.key_pressed = True
            self.signalReceived = False
        elif keyboard.is_pressed('a') and not self.key_pressed and not self.signalReceived:
            pass
        if not keyboard.is_pressed('a'):
            self.key_pressed = False

if __name__ == '__main__':
    try:
        start = Recorder()
    except rospy.ROSInterruptException:
        pass
  
