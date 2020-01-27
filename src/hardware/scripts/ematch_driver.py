#! /usr/bin/env python

# ematch_driver.py

import rospy
from std_msgs.msg import Empty
import Adafruit_BBIO.GPIO as gpio
import time
import signal
import sys
import random

def signal_handler(sig, frame):
    gpio.cleanup()
    exit(0)

def recieve_command(empty):

    rospy.loginfo("Firing e-match.")

    for i in LED:
        gpio.output(i, gpio.HIGH)

    time.sleep(2)

    for i in LED:
        gpio.output(i, gpio.LOW)

signal.signal(signal.SIGINT, signal_handler)

gpio.cleanup()
# test built in LEDs on BeagleBone Blue
LED =  [ "USR0" , "USR1", "USR2", "USR3" ]

for i in LED:
    gpio.setup(i, gpio.OUT)

rospy.init_node("ematch_driver");
sys.argv = rospy.myargv(argv=sys.argv)
if len(sys.argv) is not 2:
    rospy.logerr("Requires control pin provided in args")
    exit()

ctrl_pin = sys.argv[1]
rospy.loginfo("Starting e-match driver on pin " + ctrl_pin)
try:
    gpio.setup(ctrl_pin, gpio.OUT)
except:
    sleep = random.randint(1, 10)
    rospy.loginfo("Failed to configure. Waiting for " + str(sleep) + " seconds")
    rospy.sleep(sleep)
    try:
        gpio.setup(ctrl_pin, gpio.OUT)
    except:
        rospy.logerr("Failed to configure. Exiting.")
        exit()
gpio.output(ctrl_pin, gpio.LOW)

name = rospy.get_name()
rospy.Subscriber(name + "/command", Empty, recieve_command);
rospy.loginfo("Success.")
rospy.spin()

