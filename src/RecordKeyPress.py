#!/usr/bin/env python3

import keyboard
import rospy
from std_msgs.msg import *

rospy.init_node('RecordKeypress')
Empty_Publisher = rospy.Publisher("/KinovaAR/save", Empty, queue_size=1)
DRT_Publisher = rospy.Publisher("/KinovaAR/DRT_return", Empty, queue_size=1)

record_check = True

while True:
    if keyboard.is_pressed('space'):
        print("Spaced Pressed!")
        DRT_Publisher.publish(Empty())

    if keyboard.read_key() == 'n':
        record_check = True

    if keyboard.read_key() == 's' and record_check:
        record_check = False

        Empty_Publisher.publish()

