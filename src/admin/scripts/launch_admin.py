#! /usr/bin/env python

import rospy
from std_msgs.msg import String, UInt8, Bool, Float32

def get_command(message):

    command = message.data

    if command == "launch":
        rospy.logwarn("Beginning launch.")


rospy.init_node("launch_admin")

rospy.Subscriber("/commands", String, get_command)

rospy.spin()
