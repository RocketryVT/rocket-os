#! /usr/bin/env python

import rospy
from std_msgs.msg import String, UInt8
import re

global_readiness_level = 0
max_readiness_level = 10
level_whitelist = []
for i in range(0, max_readiness_level+1):
    level_whitelist.append(set())

def publish_readiness_level(event):

    pub_level.publish(global_readiness_level)

def print_help_text():

    unique_commands = set()
    for level in level_whitelist:
        for cmd in level:
            unique_commands.add(cmd)

    message = "\n\n    " + "Available commands are...".ljust(40)
    for i in range(0, max_readiness_level+1):
        message += str(i).ljust(3)
    message += "\n"
    for cmd in sorted(unique_commands):
        message += "    " + cmd.ljust(40)
        for level in level_whitelist:
            if cmd in level:
                message += "[*]"
            else:
                message += "   "
        message += "\n"
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

    rospy.logwarn("Command doesn't match any patterns in the current whitelist.")

rospy.init_node("readiness_admin", log_level=rospy.DEBUG)

commands = None
try:
    commands = rospy.get_param("/commands")
except:
    rospy.logerr("Failed to get commands from parameter server. Exiting.")
    rospy.signal_shutdown("Parameters unavailable.")
    exit()


for dict in commands:
    cmd = dict.keys()[0]
    privelage = dict[cmd]
    if privelage == "all":
        for s in level_whitelist:
            s.add(cmd)
    else:
        for lv in privelage:
            if lv <= max_readiness_level:
                level_whitelist[lv].add(cmd)

rospy.logdebug(str(level_whitelist))

rospy.Subscriber("/requested_commands", String, get_requested_command)
pub_level = rospy.Publisher("/readiness_level", UInt8, queue_size=10)
pub_command = rospy.Publisher("/commands", String, queue_size=10)

rospy.Timer(rospy.Duration(1), publish_readiness_level)

rospy.spin()

