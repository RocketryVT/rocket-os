#! /usr/bin/env python

# motor_driver.py

import rospy
from std_msgs.msg import UInt8

STOP = 0
CLOCKWISE = 1
COUNTERCLOCKWISE = 2

def recieve_command(command):

	if command.data is STOP:
		rospy.loginfo("Stopping the motor")

	elif command.data is CLOCKWISE:
		rospy.loginfo("Turning the motor clockwise")

	elif command.data is COUNTERCLOCKWISE:
		rospy.loginfo("Turning the motor counterclockwise")

rospy.init_node("motor_driver");
name = rospy.get_name()

rospy.Subscriber(name + "/command", UInt8, recieve_command);

rospy.loginfo("Starting DC motor driver.")

rospy.spin()

