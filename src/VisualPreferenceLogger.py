#!/usr/bin/env python

import json
import os
import rospy
from std_msgs.msg import *

class visualrecorder:

    def __init__(self, participant_id):
        
        rospy.init_node('RecordKeypress')
        self.plan_sub = rospy.Subscriber("/KinovaAR/Visual/Plan", Int32, self.plan_change_callback)
        self.visual_sub = rospy.Subscriber("/KinovaAR/Visual/Type", Int32, self.visual_change_callback)
        self.current_pair_sub = rospy.Subscriber("/KinovaAR/TrajectoryPair", Int32, self.pairing_callback)

        self.participant = participant_id
        self.visual_plan = 0
        self.visual_type = 0
        self.pairing = -1
        self.rostime = 0

        self.dictionary = {}
        rospy.spin()

        # Check if a file already exists
  
    def save_file(self):
        file_location = "/home/andre/PreferenceStudy/PreferenceStudy/Participants/"
        file_path = os.path.join(file_location, str(self.participant) + ".txt")

        if not os.path.exists(file_location):
            print("Folder path does not exist. Creating the folder...")
            try:
                os.makedirs(file_location)
            except OSError:
                print("Creation of the folder " +  file_location + " failed.")
                return

        try:
            with open(file_path, 'w') as file:
                json.dump(self.dictionary, file)
                
        except OSError:
            print("Creation of the file " + self.participant + ".txt failed.")

    def pairing_callback(self, data):
        self.pairing = data.data
        self.rostime = rospy.Time.now().to_sec()
        self.dictionary[str(self.pairing) ] = [self.visual_type, self.visual_plan, self.rostime]
        self.save_file()
        print("Trajectory Change.")

    def plan_change_callback(self, data):
        self.visual_plan = data.data
        self.rostime = rospy.Time.now().to_sec()
        self.dictionary[str(self.pairing) ] = [self.visual_type, self.visual_plan, self.rostime]
        self.save_file()
        print("Plan Change.")

    def visual_change_callback(self, data):
        self.visual_type = data.data
        self.rostime = rospy.Time.now().to_sec()
        self.dictionary[str(self.pairing) ] = [self.visual_type, self.visual_plan, self.rostime]
        self.save_file()
        print("Type Change.")

if __name__ == '__main__':
    inputval = input("Enter starting participant id: ")
    if int(inputval) > 0:
        visualrecorder(int(inputval))    
    else:
        print("Number must be 1 or larger")
    
