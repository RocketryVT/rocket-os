#! /usr/bin/env python

import rospy
from std_msgs.msg import String, UInt8

global_permission_level = 0

def publish_permission_level(event):

    publisher.publish(global_permission_level)

def get_command(message):

    global global_permission_level
    command = message.data

    if command == "echo permission level":
        rospy.loginfo("Current permission level is " + str(global_permission_level) + ".")

    elif command == "elevate permission level":
        global_permission_level = global_permission_level + 1
        rospy.logwarn("Elevated permission level to " + str(global_permission_level) + ".")

    elif command == "reduce permission level":
        global_permission_level = global_permission_level - 1
        rospy.logwarn("Reduced permission level to " + str(global_permission_level) + ".")


rospy.init_node("permission_admin")

rospy.Subscriber("/commands", String, get_command)

publisher = rospy.Publisher("/permission_level", UInt8, queue_size=10)

rospy.Timer(rospy.Duration(1), publish_permission_level)

rospy.spin()
