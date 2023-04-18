#!/usr/bin/env python3

import rosbag
import rospy

from moveit_msgs.msg import *

rospy.init_node('RosbagManager')

# directory_location = "/home/andre/catkin_ws/src/ROS_KinovaGen3LiteAR/Trajectories/"
directory_location = '/home/andre/catkin_ws/src/kinova_study/src/Trajectories/'
rosbagfileName1 = "0"
rosbagfileName2 = "1"

filetype = ".bag"

file1 = directory_location + rosbagfileName1 + filetype
file2 = directory_location + rosbagfileName2 + filetype

Trajectory1 = DisplayTrajectory()
Trajectory2 = DisplayTrajectory()

bag_in1 = rosbag.Bag(file1)
bag_in2 = rosbag.Bag(file2)

msg_counters = {} # empty dict
total_count = 0

FirstTrajectory_Publisher = rospy.Publisher("/KinovaAR/FirstTrajectoryDisplay", DisplayTrajectory, queue_size=1)
SecondTrajectory_Publisher = rospy.Publisher("/KinovaAR/SecondTrajectory", DisplayTrajectory, queue_size=1)

for topic, msg, t in bag_in1.read_messages():
    print("\n# =======================================")
    total_count += 1
    no_msgs_found = False
    # Keep track of individual message counters for each message type
    if topic not in msg_counters:
        msg_counters[topic] = 1
    else:
        msg_counters[topic] += 1

    if (topic == "/KinovaAR/FirstTrajectory"):
        Trajectory1 = msg

    # Print topic name and message receipt info
    print("# topic:           " + topic)
    print("# msg_count:       %u" % msg_counters[topic])
    # the comma at the end prevents the newline char being printed at the end in Python2; see:
    # https://www.geeksforgeeks.org/print-without-newline-python/
    print("# timestamp (sec): {:.9f}".format(t.to_sec())),
    print("# - - -")

for topic, msg, t in bag_in2.read_messages():
    print("\n# =======================================")
    total_count += 1
    no_msgs_found = False
    # Keep track of individual message counters for each message type
    if topic not in msg_counters:
        msg_counters[topic] = 1
    else:
        msg_counters[topic] += 1

    if (topic == "/KinovaAR/FirstTrajectory"):
        Trajectory2 = msg


FirstTrajectory_Publisher.publish(Trajectory1)
SecondTrajectory_Publisher.publish(Trajectory2)
