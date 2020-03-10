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

    gpio.output(ctrl_pin_a, gpio.HIGH)
    time.sleep(delay)
    gpio.output(ctrl_pin_b, gpio.HIGH)

    time.sleep(2)

    gpio.output(ctrl_pin_a, gpio.LOW)
    gpio.output(ctrl_pin_b, gpio.LOW)


signal.signal(signal.SIGINT, signal_handler)

gpio.cleanup()

rospy.init_node("ematch_driver");
name = rospy.get_name()

try:
    ctrl_pin_a = rospy.get_param(name + "/pin_a")
    ctrl_pin_b = rospy.get_param(name + "/pin_b")
    delay = rospy.get_param(name + "/delay")
except:
    rospy.logerr("Failed to retrieve configuration from rosparam server.")
    rospy.signal_shutdown("Unavailable config.")
    exit()
rospy.loginfo("Starting e-match driver on pins " + ctrl_pin_a + ", " + ctrl_pin_b)
rospy.loginfo("Using a delay of {} seconds.".format(delay))

success = False
max_attempts = 10
for i in range(max_attempts):

    try:
        gpio.setup(ctrl_pin_a, gpio.OUT)
        gpio.setup(ctrl_pin_b, gpio.OUT)
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

gpio.output(ctrl_pin_a, gpio.LOW)
gpio.output(ctrl_pin_b, gpio.LOW)

rospy.Subscriber(name + "/command", Empty, recieve_command);
rospy.loginfo("Success.")
rospy.spin()

