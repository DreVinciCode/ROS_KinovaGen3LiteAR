#!/usr/bin/env python3

import keyboard
import rospy
from std_msgs.msg import Empty

class Recorder(object):
    def __init__(self):

        rospy.init_node('RecordKeypress')
        Empty_Publisher = rospy.Publisher("/KinovaAR/save", Empty, queue_size=1)
        self.record_check = True

        while True:   
            if keyboard.read_key() == 'n':
                self.record_check = True

            if keyboard.read_key() == 's' and self.record_check:
                self.record_check = False
                Empty_Publisher.publish()

if __name__ == '__main__':
    try:
        start = Recorder()
    except rospy.ROSInterruptException:
        pass