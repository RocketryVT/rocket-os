#! /usr/bin/env python

import rospy
from std_msgs.msg import String, UInt8
import re

global_readiness_level = 0

# command whitelist
persistent_whitelist = ["read data", "read data .*", "stop data", "system .*", "print whitelist", "print readiness level", "set readiness [0-9]+"]
level_whitelist = [
    ["idiot check"],
    ["close vent valve", "close ignition valve", "abort"],
    ["begin fill", "end fill", "open vent valve", "close vent valve", "crack vent valve", "abort"],
    ["end fill", "open vent valve", "close vent valve", "crack vent valve", "fill disconnect", "abort"],
    ["open vent valve", "crack vent valve", "close vent valve", "idiot check part two", "abort"],
    ["open vent valve", "crack vent valve", "close vent valve", "arm rocket", "abort"],
    ["get ready to rumble", "abort", "rollback"],
    ["launch the rocket", "abort", "rollback"],
    ["abort", ".*"],
    [""],
    [".*"]
]

def publish_readiness_level(event):

    pub_level.publish(global_readiness_level)

def print_help_text():

    message = "\n\n\tAvailable commands are...\n"
    message += "\t" + "Persistent:".ljust(12) + ", ".join(persistent_whitelist) + "\n"
    for i in range(0, len(level_whitelist)):
        message += "\t" + ("RL-" + str(i) + ":").ljust(12) + ", ".join(level_whitelist[i]) + "\n"
    rospy.loginfo(message)

def receive_command(string):

    global global_readiness_level
    if string == "print whitelist":

        print_help_text()

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
            rospy.logdebug("Command matches current pattern: " + regex + ", " + command)
            pub_command.publish(command)
            receive_command(command)
            return

    for regex in persistent_whitelist:
        if bool(re.match(re.compile("^" + regex + "$"), command)):
            rospy.logdebug("Command matches persistent pattern: " + regex + ", " + command)
            pub_command.publish(command)
            receive_command(command)
            return

    rospy.loginfo("Command doesn't match any patterns in the current whitelist.")
    print_help_text()

rospy.init_node("readiness_admin", log_level=rospy.DEBUG)
rospy.Subscriber("/requested_commands", String, get_requested_command)
pub_level = rospy.Publisher("/readiness_level", UInt8, queue_size=10)
pub_command = rospy.Publisher("/commands", String, queue_size=10)

rospy.Timer(rospy.Duration(1), publish_readiness_level)

rospy.spin()
