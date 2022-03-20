#! /usr/bin/env python3

'''
Commands Sender: Sets up a prompt for users to input commands
                and publishes the user inputs to "/commands" topic.

'''

import rospy
from std_msgs.msg import String
import signal

def signal_handler(frame, sig):

	'''
		Closes the Commands Sender node.
	'''

    print("")
    exit()

#If Key Interrupt (Ctrl-C or Delete) detected, call signal_handler
signal.signal(signal.SIGINT, signal_handler)

rospy.init_node("command_sender")
pub = rospy.Publisher("/commands", String, queue_size=10)

while True:

    print(">>"),
    input = raw_input()
    pub.publish(input)
