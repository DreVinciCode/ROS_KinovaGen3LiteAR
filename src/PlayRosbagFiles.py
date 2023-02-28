#!/usr/bin/env python3

import rosbag
import rospy

from moveit_msgs.msg import *


rospy.init_node('RosbagManager')


directory_location = "/home/andre/catkin_ws/src/ROS_KinovaGen3LiteAR/"
rosbagfileName = "rosbagTest"
filetype = ".bag"


file = directory_location + rosbagfileName + filetype

Trajectory = DisplayTrajectory()

bag_in = rosbag.Bag(file)
msg_counters = {} # empty dict
total_count = 0

FirstTrajectory_Publisher = rospy.Publisher("/KinovaAR/FirstTrajectory", DisplayTrajectory, queue_size=1)



for topic, msg, t in bag_in.read_messages():
    print("\n# =======================================")
    total_count += 1
    no_msgs_found = False
    # Keep track of individual message counters for each message type
    if topic not in msg_counters:
        msg_counters[topic] = 1
    else:
        msg_counters[topic] += 1

    # Print topic name and message receipt info
    print("# topic:           " + topic)
    print("# msg_count:       %u" % msg_counters[topic])
    # the comma at the end prevents the newline char being printed at the end in Python2; see:
    # https://www.geeksforgeeks.org/print-without-newline-python/
    print("# timestamp (sec): {:.9f}".format(t.to_sec())),
    print("# - - -")

    # Print the message
    #  print(msg)

# for topic, msg in bag_in.read_messages():
#     print(msg)

for topic in bag_in.read_messages():
    for msg in topic:
        pass


FirstTrajectory_Publisher.publish(Trajectory)

