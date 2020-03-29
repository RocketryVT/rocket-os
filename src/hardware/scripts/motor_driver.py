#! /usr/bin/env python

# motor_driver.py

import rospy
from hardware.msg import DriverCommand
import sys
import Adafruit_BBIO.GPIO as gpio
import random


def recieve_command(msg):

    if msg.command == msg.RELEASE:
        rospy.loginfo("Command from " + msg.source + ": release the motor")
        rospy.loginfo("NOT IMPLEMENTED")

    if msg.command == msg.STOP:
        rospy.loginfo("Command from " + msg.source + ": stop the motor")
        gpio.output(cw_pin, gpio.LOW)
        gpio.output(ccw_pin, gpio.LOW)

    elif msg.command is msg.CLOSE:
        rospy.loginfo("Command from " + msg.source + ": close the motor")
        gpio.output(cw_pin, gpio.HIGH)
        gpio.output(ccw_pin, gpio.LOW)

    elif msg.command is msg.OPEN:
        rospy.loginfo("Command from " + msg.source + ": open the motor")
        gpio.output(cw_pin, gpio.LOW)
        gpio.output(ccw_pin, gpio.HIGH)

    elif msg.command is msg.PULSE_CLOSE:
        rospy.loginfo("Command from " + msg.source + ": pulse-close " +
            "the motor for {} seconds".format(dur))
        rospy.loginfo("NOT IMPLEMENTED")

    elif msg.command is msg.PULSE_OPEN:
        dur = msg.pulse.to_sec()
        rospy.loginfo("Command from " + msg.source + ": pulse-open " +
            "the motor for {} seconds".format(dur))
        rospy.loginfo("NOT IMPLEMENTED")


rospy.init_node("motor_driver", log_level=rospy.DEBUG);
name = rospy.get_name()

try:
    cw_pin = rospy.get_param(name + "/pin_a")
    ccw_pin = rospy.get_param(name + "/pin_b")
except:
    rospy.logerr("Failed to retrieve pin config from rosparam server.")
    rospy.signal_shutdown("Unavailable pin config.")
    exit()

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

rospy.Subscriber(name, DriverCommand, recieve_command);
rospy.loginfo("Success.")
rospy.spin()

