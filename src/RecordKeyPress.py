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
        if keyboard.is_pressed('space') and not self.key_pressed and self.signalReceived:
            self.DRT_Publisher.publish(Empty())
            self.key_pressed = True
            self.signalReceived = False
            print("Good hit")
        elif keyboard.is_pressed('space') and not self.key_pressed and not self.signalReceived:
            print("Cant press now.")

        if not keyboard.is_pressed('space'):
            self.key_pressed = False
            
        rospy.sleep(0.1)  # Adjust the sleep duration as needed to reduce CPU usage

if __name__ == '__main__':
    try:
        start = Recorder()
    except rospy.ROSInterruptException:
        pass
  
    # if keyboard.read_key() == 'space' and not key_pressed:
    #     DRT_Publisher.publish(Empty())    
    #     key_pressed = True

    # if not keyboard.is_pressed('space'):
    #     key_pressed = False

    # if keyboard.read_key() == 'p':
    #     keyboard.press_and_release('p')
    #     print("_")
        
    # if keyboard.read_key() == 'n':
    #     record_check = True

    # if keyboard.read_key() == 's' and record_check:
    #     record_check = False

    #     Empty_Publisher.publish()