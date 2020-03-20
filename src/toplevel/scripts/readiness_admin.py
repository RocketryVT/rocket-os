#! /usr/bin/env python

import rospy
from std_msgs.msg import String, UInt8
import re


def publish_readiness_level(event):

    pub_level.publish(global_readiness_level)


def print_help_text():

    message = "Available commands are...\n\n"
    for id, cmd, permission in whitelist:
        if global_readiness_level in permission or backdoor:
            message += "    " + cmd + "\n"
    rospy.loginfo(message)


def set_readiness_level(new_level):

    global global_readiness_level
    if new_level > max_readiness_level or new_level < 0:
        rospy.logwarn("Can't set readiness level -- " +
            "must be in [0, {}]".format(max_readiness_level))
    else:
        global_readiness_level = new_level
        rospy.loginfo("Set readiness level to " + str(new_level))


def receive_command(string):

    global global_readiness_level

    if string == "print whitelist":

        print_help_text()

    elif string == "print readiness level":

        rospy.loginfo("Current readiness level is " + str(global_readiness_level))

    elif string == "elevate readiness":

        set_readiness_level(global_readiness_level + 1)

    elif string == "reduce readiness":

        set_readiness_level(global_readiness_level - 1)


def get_requested_command(message):

    global backdoor
    command = message.data

    if command == "toggle backdoor":
        backdoor = not backdoor
        if backdoor:
            rospy.logwarn("Enabled backdoor. All commands now permitted.")
        else:
            rospy.loginfo("Backdoor disabled.")
        return

    matches = []
    for id, regex, permission in whitelist:
        if bool(re.match(re.compile("^" + regex + "$"), command)) and global_readiness_level in permission:
            matches.append(regex)

    if matches:
        rospy.logdebug("Command '{}' matches these patterns: {}".format(command, matches))
        best_match = matches[-1]
        pub_command.publish(command)
        receive_command(command)
    elif backdoor:
        rospy.logdebug("Backdoor allowing otherwise disallowed command.")
        pub_command.publish(command)
        receive_command(command)
    else:
        rospy.logwarn("Command doesn't match any patterns in the current whitelist.")


if __name__ == "__main__":

    rospy.init_node("readiness_admin", log_level=rospy.DEBUG)

    global_readiness_level = 0
    max_readiness_level = 9
    whitelist = []
    backdoor = False

    commands = None
    try:
        commands = rospy.get_param("/commands")
    except:
        rospy.logerr("Failed to get commands from parameter server. Exiting.")
        rospy.signal_shutdown("Parameters unavailable.")
        exit()

    for id, dict in enumerate(commands):
        cmd = dict.keys()[0]
        privelage = dict[cmd]
        if privelage == "all":
            privelage = range(0, max_readiness_level + 1)
        whitelist.append((id, cmd, privelage))

    rospy.Subscriber("/requested_commands", String, get_requested_command)
    pub_level = rospy.Publisher("/readiness_level", UInt8, queue_size=10)
    pub_command = rospy.Publisher("/commands", String, queue_size=10)

    rospy.Timer(rospy.Duration(1), publish_readiness_level)

    rospy.spin()

