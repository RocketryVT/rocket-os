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

    secs = rospy.Time.now().to_sec() % 10

    if (nominal_state and secs < 4) and not current_state:
        rospy.loginfo("Opening the solenoid")
        current_state = True
        for i in LED:
            gpio.output(i, gpio.HIGH)

    elif not (nominal_state and secs < 4) and current_state:
        rospy.loginfo("Closing the solenoid")
        current_state = False
        for i in LED:
            gpio.output(i, gpio.LOW)


gpio.cleanup()
LED =  [ "USR0" , "USR1", "USR2", "USR3" ]
for i in LED:
    gpio.setup(i, gpio.OUT)

rospy.init_node("solenoid_driver");
sys.argv = rospy.myargv(argv=sys.argv)
if len(sys.argv) is not 2:
    rospy.logerr("Requires a control pin via args")
    exit()

ctrl_pin = sys.argv[1]
rospy.loginfo("Starting solenoid driver on pin " + ctrl_pin)
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
rospy.Subscriber(name + "/command", Bool, recieve_command);
rospy.Timer(rospy.Duration(0.5), control_loop)
rospy.loginfo("Success.")
rospy.spin()

