#!/usr/bin/env python

import os
import rospy
from std_msgs.msg import *

class visualrecorder:

    def __init__(self, participant_id):
        
        file_location = "/home/andre/PreferenceStudy/PreferenceStudy/Participants/"
        file_path = os.path.join(file_location, str(participant_id) + ".txt")

        if not os.path.exists(file_location):
            print("Folder path does not exist. Creating the folder...")
            try:
                os.makedirs(file_location)
            except OSError:
                print("Creation of the folder " +  file_location + " failed.")
                return

        try:
            with open(file_path, 'w') as file:
                # You can write content to the file if needed
                # file.write("Hello,"  + str(participant_id) + "!")
                print("File " + str(participant_id) + ".txt created and saved in " + file_location )
        except OSError:
            print("Creation of the file " + participant_id + ".txt failed.")


        rospy.init_node('RecordKeypress')
        self.plan_sub = rospy.Subscriber("/KinovaAR/Visual/Plan", Int32, self.plan_change_callback)
        self.visual_sub = rospy.Subscriber("/KinovaAR/Visual/Type", Int32, self.visual_change_callback)

        rospy.spin()
  
  
    def plan_change_callback(self, data):
        pass




    def visual_change_callback(self, data):
        pass


if __name__ == '__main__':
    inputval = input("Enter starting participant id: ")
    if int(inputval) > 0:
        visualrecorder(int(inputval))    
    else:
        print("Number must be 1 or larger")
    
