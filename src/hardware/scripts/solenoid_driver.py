#! /usr/bin/env python

# solenoid_driver.py

import rospy
from std_msgs.msg import Bool
import sys
import Adafruit_BBIO.GPIO as gpio
import time
import signal
import random

nominal_state = False
current_state = False

def signal_handler(sig, frame):
    gpio.cleanup()
    exit(0)

def recieve_command(command):

    global nominal_state
    if command.data:
        rospy.loginfo("Enable solenoid open cycle")
        nominal_state = True

    else:
        rospy.loginfo("Disable solenoid open cycle")
        nominal_state = False

def control_loop(event):

    global nominal_state
    global current_state

    secs = rospy.Time.now().to_sec() % (opened_secs + closed_secs)

    if (nominal_state and secs < opened_secs) and not current_state:
        rospy.loginfo("Opening the solenoid")
        current_state = True
        gpio.output(ctrl_pin, gpio.HIGH)

    elif not (nominal_state and secs < opened_secs) and current_state:
        rospy.loginfo("Closing the solenoid")
        current_state = False
        gpio.output(ctrl_pin, gpio.LOW)


gpio.cleanup()

rospy.init_node("solenoid_driver");
name = rospy.get_name()
name = rospy.get_name()
try:
    ctrl_pin = rospy.get_param(name + "/pin")
    opened_secs = rospy.get_param(name + "/opened")
    closed_secs = rospy.get_param(name + "/closed")
except:
    rospy.logerr("Failed to retrieve configuration from rosparam server.")
    rospy.signal_shutdown("Unavailable config.")
    exit()
rospy.loginfo("Starting solenoid driver on pin " + ctrl_pin)
rospy.loginfo("Using {} s, {} s open-close cycle.".format(opened_secs, closed_secs))

success = False
max_attempts = 10
for i in range(max_attempts):

    try:
        gpio.setup(ctrl_pin, gpio.OUT)
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

gpio.output(ctrl_pin, gpio.LOW)

rospy.Subscriber(name + "/command", Bool, recieve_command);
rospy.Timer(rospy.Duration(0.5), control_loop)


rospy.loginfo("Success.")
rospy.spin()

