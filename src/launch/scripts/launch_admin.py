#! /usr/bin/env python

import rospy
from std_msgs.msg import String, UInt8, Bool, Float32

def get_command(message):

    global fill_ongoing
    command = message.data

    if command == "launch":
        rospy.logwarn("Beginning launch.")
        fill_ongoing = True

    if command == "yes"


rospy.init_node("launch_admin")

rospy.Subscriber("/commands", String, get_command)

rospy.Timer(rospy.Duration(10), print_relevant_data)

rospy.spin()
