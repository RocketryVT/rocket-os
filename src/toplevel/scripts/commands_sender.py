#! /usr/bin/env python

'''
Commands Sender: ????????????

'''

import rospy
from std_msgs.msg import String
import signal

def signal_handler(frame, sig):

	'''
		Description Here
	'''

    print("")
    exit()

signal.signal(signal.SIGINT, signal_handler)

rospy.init_node("command_sender")
pub = rospy.Publisher("/commands", String, queue_size=10)

while True:

    print(">>"),
    input = raw_input()
    pub.publish(input)
