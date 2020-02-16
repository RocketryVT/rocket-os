#! /usr/bin/env python

# motor_driver.py

import rospy
from std_msgs.msg import UInt8
import sys
import Adafruit_BBIO.GPIO as gpio
import random

STOP = 0
CLOCKWISE = 1
COUNTERCLOCKWISE = 2

def recieve_command(command):

    if command.data is STOP:
        rospy.loginfo("Stopping the motor")
        gpio.output(cw_pin, gpio.LOW)
        gpio.output(ccw_pin, gpio.LOW)

    elif command.data is CLOCKWISE:
        rospy.loginfo("Turning the motor clockwise")
        gpio.output(cw_pin, gpio.HIGH)
        gpio.output(ccw_pin, gpio.LOW)

    elif command.data is COUNTERCLOCKWISE:
        rospy.loginfo("Turning the motor counterclockwise")
        gpio.output(cw_pin, gpio.LOW)
        gpio.output(ccw_pin, gpio.HIGH)


rospy.init_node("motor_driver", log_level=rospy.DEBUG);
sys.argv = rospy.myargv(argv=sys.argv)
if len(sys.argv) is not 3:
    rospy.logerr("Requires two pins as arguments: <cw> <ccw>")
    exit()
cw_pin = sys.argv[1]
ccw_pin = sys.argv[2]
rospy.loginfo("Starting DC motor driver on pins " + cw_pin + ", " + ccw_pin)

success = False
max_attempts = 10
for i in range(max_attempts):

    try:
        gpio.setup(cw_pin, gpio.OUT)
        gpio.setup(ccw_pin, gpio.OUT)
        success = True
    except:
        sleep = random.randint(1, 20)
        rospy.logwarn(str(i) + ": Failed to configure. Waiting for " + str(sleep) + " seconds before reattempt.")
        rospy.sleep(sleep)

    if success:
        break

if not success:
    rospy.logerr("Failed to configure after " + str(max_attempts) + " attempts.")
    exit()

gpio.output(cw_pin, gpio.LOW)
gpio.output(ccw_pin, gpio.LOW)
name = rospy.get_name()
rospy.Subscriber(name + "/command", UInt8, recieve_command);
rospy.loginfo("Success.")
rospy.spin()

