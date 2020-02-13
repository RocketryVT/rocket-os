#!/usr/bin/env python

import rospy
import subprocess
import signal, fcntl, os
from std_msgs.msg import String

rospy.init_node("exec", log_level=rospy.DEBUG)

timeout = 10

def get_command(cmd):

    global timeout
    tokens = cmd.data.split(" ")

    if tokens[0] == "shutdown":

        rospy.signal_shutdown("Command recieved")

    if tokens[0] == "timeout":

        if len(tokens) == 1:
            rospy.loginfo("Current timeout is {} seconds.".format(timeout))
        else:
            try:
                timeout = float(tokens[1])
                rospy.loginfo("Set command timeout to {} seconds.".
                    format(timeout))
            except:
                rospy.logerr("Error parsing command - expecting: timeout %f")

    if tokens[0] == "fork":
        rospy.loginfo("$ " + " ".join(tokens[1:]))
        process = subprocess.Popen(" ".join(tokens[1:]),
            shell=True)
        rospy.loginfo("Done.")


    if tokens[0] == "system":
        rospy.loginfo("$ " + " ".join(tokens[1:]))
        p = subprocess.Popen(" ".join(tokens[1:]),
            shell=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE)

        start = rospy.get_rostime()

        def make_nonblocking(filestream):
            fd = filestream.fileno()
            fl = fcntl.fcntl(fd, fcntl.F_GETFL)
            fcntl.fcntl(fd, fcntl.F_SETFL, fl | os.O_NONBLOCK)

        make_nonblocking(p.stdout)
        make_nonblocking(p.stderr)

        stdout, stderr = "", ""
        while p.poll() is None:

            now = rospy.get_rostime()
            if now - start > rospy.Duration(timeout):
                p.kill()
                try:
                    stdout = p.stdout.read()
                except:
                    pass
                try:
                    stderr = p.stderr.read()
                except:
                    pass

                if len(stdout.split("\n")) > 2:
                    rospy.loginfo("\n" + stdout)
                elif stdout:
                    rospy.loginfo(stdout)
                if len(stderr.split("\n")) > 2:
                    rospy.logerr("\n" + stderr)
                elif stderr:
                    rospy.logerr(stderr)
                rospy.logwarn("""Command timed out after"""
                    """ {} seconds.""".format(timeout))
                return

        stdout, stderr = p.communicate()

        if len(stdout.split("\n")) > 2:
            rospy.loginfo("\n" + stdout)
        elif stdout:
            rospy.loginfo(stdout)
        if len(stderr.split("\n")) > 2:
            rospy.logerr("\n" + stderr)
        elif stderr:
            rospy.logerr(stderr)
        exit = p.poll()
        rospy.loginfo("Finished with exit code " + str(exit))

sub = rospy.Subscriber("/commands", String, get_command)

rospy.spin()
