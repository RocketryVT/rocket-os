#! /usr/bin/env python

import rospy
from std_msgs.msg import String, UInt8, Empty

def decrement_countdown(event):

    global remaining
    
    if remaining == 0:
        rospy.logwarn("T - {} seconds.".format(remaining))
        no_turning_back()
        return

    rospy.logwarn("T - {} seconds.".format(remaining))
    remaining -= 1


def no_turning_back():

    # execute this if we definitely want to launch
    stop_timer()
    # seriously, there's no way to stop the funk
    # if this function is called
    rospy.logwarn("Executing launch sequence.")
    rospy.sleep(0.5)

    abort_valve.publish(1) # close abort valve
    rospy.sleep(1)
    injection_valve.publish(2) # open injection valve
    rospy.sleep(0.4)
    ematch.publish() # fire ematches
    
    pub_commands.publish("system fortune | cowsay")


def start_timer():

    global remaining
    global countdown_timer
    remaining = countdown_maximum
    countdown_timer = rospy.Timer(rospy.Duration(1), decrement_countdown)


def stop_timer():

    global remaining
    global countdown_timer
    remaining = countdown_maximum
    countdown_timer.shutdown()
    countdown_timer = None


def get_command(message):

    command = message.data

    if command == "launch" and countdown_timer is None:
        rospy.logwarn("Beginning launch countdown. Any user input will abort the launch.")
        start_timer()

    elif countdown_timer is not None:
        stop_timer()
        rospy.logwarn("User input detected -- launch aborted.")


if __name__ == "__main__":

    countdown_maximum = 10 # number of seconds to count down from
    remaining = countdown_maximum
    countdown_timer = None

    rospy.init_node("launch_admin")
    rospy.Subscriber("/commands", String, get_command)
    pub_commands = rospy.Publisher("/requested_commands", String, queue_size=10)

    # launch involves injection valve, ematch, abort valve
    injection_valve = rospy.Publisher("/hardware/injection_valve/request", UInt8, queue_size=10)
    abort_valve = rospy.Publisher("/hardware/abort_valve/request", UInt8, queue_size=10)
    ematch = rospy.Publisher("/hardware/ematch/command", Empty, queue_size=10)

    rospy.spin()

