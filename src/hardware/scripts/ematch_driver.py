#! /usr/bin/env python

# ematch_driver.py

import rospy
from std_msgs.msg import Empty
import Adafruit_BBIO.GPIO as gpio
import time
import signal

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
name = rospy.get_name()

rospy.Subscriber(name + "/command", Empty, recieve_command);

rospy.loginfo("Starting e-match driver.")

rospy.spin()

