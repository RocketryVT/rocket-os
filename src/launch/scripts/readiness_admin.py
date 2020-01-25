#! /usr/bin/env python

import rospy
from std_msgs.msg import String, UInt8
import re

global_readiness_level = 0

# command whitelist
persistent_whitelist = ["system .*", "print whitelist", "print readiness level", "set readiness [0-9]+"]
level_whitelist = [
    ["read data", "idiot check"],
    ["read data", "close vent valve", "close ignition valve", "abort"],
    ["read data", "begin fill", "end fill", "open vent valve", "close vent valve", "crack vent valve", "abort"],
    ["read data", "end fill", "open vent valve", "close vent valve", "crack vent valve", "fill disconnect", "abort"],
    ["read data", "open vent valve", "crack vent valve", "close vent valve", "idiot check part two", "abort"],
    ["read data", "open vent valve", "crack vent valve", "close vent valve", "arm rocket", "abort"],
    ["read data", "get ready to rumble", "abort", "rollback"],
    ["read data", "launch the rocket", "abort", "rollback"],
    ["abort", ".*"],
    [""],
    [".*"]
]

def publish_readiness_level(event):

    pub_level.publish(global_readiness_level)

def receive_command(string):

    global global_readiness_level
    if string == "print whitelist":

        rospy.loginfo("Available commands are...")
        rospy.loginfo("Persistent: " + ", ".join(persistent_whitelist))
        for i in range(0, len(level_whitelist)):
            rospy.loginfo("RL-" + str(i) + ": " + ", ".join(level_whitelist[i]))

    elif string == "print readiness level":

        rospy.loginfo("Current readiness level is " + str(global_readiness_level))

    elif bool(re.match(re.compile("^set readiness [0-9]+"), string)):

        new_level = int(string.split()[2])

        if new_level >= len(level_whitelist) or new_level < 0:
            rospy.logwarn("Can't set readiness level -- must be in [0, " + str(len(level_whitelist) - 1) + "]")
        else:
            global_readiness_level = new_level
            rospy.loginfo("Set readiness level to " + str(new_level))

def get_requested_command(message):

    command = message.data

    for regex in level_whitelist[global_readiness_level]:
        if bool(re.match(re.compile("^" + regex + "$"), command)):
            print("Command matches current pattern: " + regex + ", " + command)
            pub_command.publish(command)
            receive_command(command)
            return

    for regex in persistent_whitelist:
        if bool(re.match(re.compile("^" + regex + "$"), command)):
            print("Command matches persistent pattern: " + regex + ", " + command)
            pub_command.publish(command)
            receive_command(command)
            return

    rospy.loginfo("Command doesn't match any patterns in the current whitelist.")
    rospy.loginfo("Available commands are...")
    rospy.loginfo("Persistent: " + ", ".join(persistent_whitelist))
    rospy.loginfo("Current: " + ", ".join(level_whitelist[global_readiness_level]))


rospy.init_node("readiness_admin")
rospy.Subscriber("/requested_commands", String, get_requested_command)
pub_level = rospy.Publisher("/readiness_level", UInt8, queue_size=10)
pub_command = rospy.Publisher("/commands", String, queue_size=10)

rospy.Timer(rospy.Duration(1), publish_readiness_level)

rospy.spin()
