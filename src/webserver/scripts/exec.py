#!/usr/bin/env python

import rospy
import subprocess
from std_msgs.msg import String

rospy.init_node("exec")

def get_command(cmd):

    tokens = cmd.data.split(" ")

    if tokens[0] == "system":
        rospy.loginfo("$ " + " ".join(tokens[1:]))
        p = subprocess.Popen(" ".join(tokens[1:]),
            shell=True,
            stderr=subprocess.PIPE,
            stdout=subprocess.PIPE)
        stdout, stderr = p.communicate()
        for line in stdout.split("\n")[0:-1]:
            rospy.loginfo(line)
        for line in stderr.split("\n")[0:-1]:
            rospy.logerr(line)

sub = rospy.Subscriber("/webserver/commands", String, get_command)

rospy.spin()

