#!/usr/bin/env python

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

        self.startTime = rospy.Time.now()

        self.entry = []
        rospy.spin()
  
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
            with open(file_path, 'a') as file:
                file.write(str(self.entry) + "\n")
                
        except OSError:
            print("Creation of the file " + self.participant + ".txt failed.")

    def pairing_callback(self, data):
        self.pairing = data.data
        rostime = (rospy.Time.now() - self.startTime).to_sec()
        self.entry = [self.pairing, self.visual_type, self.visual_plan, rostime]
        self.save_file()
        print("Trajectory Change.")

    def plan_change_callback(self, data):
        self.visual_plan = data.data
        rostime = (rospy.Time.now() - self.startTime).to_sec()
        self.entry = [self.pairing, self.visual_type, self.visual_plan, rostime]
        self.save_file()
        print("Plan Change.")

    def visual_change_callback(self, data):
        self.visual_type = data.data
        rostime = (rospy.Time.now() - self.startTime).to_sec()
        self.entry = [self.pairing, self.visual_type, self.visual_plan, rostime]
        self.save_file()
        print("Type Change.")

if __name__ == '__main__':
    file_location = "/home/andre/PreferenceStudy/PreferenceStudy/Participants/"
    inputval = input("Enter starting participant id: ")

    while True:
        file = file_location +  str(inputval) + ".txt"
        if os.path.exists(file):
            print("File already exists...")
            inputval = input("Enter starting participant id: ")

        if int(inputval) < 0:
            print("Number must be 1 or larger")
            inputval = input("Enter starting participant id: ")

        else:
            visualrecorder(int(inputval))    
            break



    
