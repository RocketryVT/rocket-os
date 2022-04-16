#!/usr/bin/env python3

'''
Exec: 
			  + Num of Functions: 2
'''

import rospy
import subprocess
import signal, fcntl, os
from std_msgs.msg import String


def get_command(cmd):
    '''
        Executes any of the following commands:

            - shutdown: Initiate shutdown process for all motor controller systems
            - timeout: Print max time child process is allowed to run before being killed
            - timeout <seconds>: Set max time child process is allowed to run before being killed
            - system <command>: Execute a Linux Bash command on child shell process & return
                                the result to the console.
            - fork <command>: Execute a Linux Bash command on child shell process but doesn't
                                wait for it to complete before returning (no console output provided)
    '''
    global timeout
    tokens = cmd.data.split(" ")

    if tokens[0] == "shutdown":

        # Initiate shutdown process
        rospy.signal_shutdown("Command recieved")

    # timeout Command
    if tokens[0] == "timeout":

        if len(tokens) == 1:
            rospy.loginfo("Current timeout is {} seconds.".format(timeout))
        
        # timeout <seconds> Command
        else:
            try:
                timeout = float(tokens[1])
                rospy.loginfo("Set command timeout to {} seconds.".
                    format(timeout))
            except:
                rospy.logerr("Error parsing command - expecting: timeout %f")

    # fork <command> Command
    if tokens[0] == "fork":
        rospy.loginfo("$ " + " ".join(tokens[1:]))
        process = subprocess.Popen(" ".join(tokens[1:]),
            shell=True)
        rospy.loginfo("Done.")

    # system <command> Command
    if tokens[0] == "system":
        rospy.loginfo("$ " + " ".join(tokens[1:]))
        p = subprocess.Popen(" ".join(tokens[1:]),
            shell=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE)

        start = rospy.get_rostime()

        def make_nonblocking(filestream):
            '''
                Prevent filestream from blocking the execution of further operations.
                (i.e. Make asynchronous)
            '''
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


if __name__ == "__main__":
    
    # Initialize Node
    rospy.init_node("exec", log_level=rospy.DEBUG)
    timeout = 10

    # Set Subscription
    sub = rospy.Subscriber("/commands", String, get_command)
    rospy.spin()
