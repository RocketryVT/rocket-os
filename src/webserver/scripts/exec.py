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

        start = rospy.get_rostime()
        while p.poll() is None:
            now = rospy.get_rostime()
            if now - start > rospy.Duration(10):
                rospy.logwarn("Command timed out after 10 seconds.")
                p.kill()
                return

        stdout, stderr = p.communicate()
        exit = p.poll()
        if len(stdout.split("\n")) > 2:
            rospy.loginfo("\n" + stdout)
        elif stdout:
            rospy.loginfo(stdout)
        if len(stderr.split("\n")) > 2:
            rospy.logerr("\n" + stderr)
        elif stderr:
            rospy.logerr(stderr)
        rospy.loginfo("Finished with exit code " + str(exit))

sub = rospy.Subscriber("/webserver/commands", String, get_command)

rospy.spin()
