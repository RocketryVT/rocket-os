#! /usr/bin/env python

# solenoid_driver.py

import rospy
import sys
import Adafruit_BBIO.GPIO as gpio
import time
import signal
import random
from hardware.msg import DriverCommand
import driverlib


def signal_handler(sig, frame):
    gpio.cleanup()
    exit(0)


def execute_solenoid_command(msg):

    global nominal_state
    if msg.command == msg.SOLENOID_ACTIVE:
        rospy.loginfo("Enable solenoid open cycle")
        nominal_state = True
    elif msg.command == msg.SOLENOID_INACTIVE:
        rospy.loginfo("Disable solenoid open cycle")
        nominal_state = False
    else:
        rospy.logwarn("Unimplemented command: " + str(msg.command))
        driverlib.nullify_command(msg)

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


if __name__ == "__main__":

    nominal_state = False
    current_state = False

    gpio.cleanup()

    rospy.init_node("solenoid_driver", log_level=rospy.DEBUG);
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
    rospy.loginfo("Using {} s, {} s open-close cycle.".format( \
        opened_secs, closed_secs))

    success = False
    max_attempts = 10
    for i in range(max_attempts):

        try:
            gpio.setup(ctrl_pin, gpio.OUT)
            success = True
        except:
            sleep = random.randint(1, 20)
            rospy.logwarn(str(i) + ": Failed to configure. " + \
                "Waiting for " + str(sleep) + " seconds before reattempt.")
            rospy.sleep(sleep)

        if success:
            break

    if not success:
        rospy.logerr("Failed to configure after " + \
            str(max_attempts) + " attempts.")
        exit()

    gpio.output(ctrl_pin, gpio.LOW)

    driverlib.callback(execute_solenoid_command)
    rospy.Subscriber(name, DriverCommand, driverlib.receive_command);
    rospy.Timer(rospy.Duration(0.5), control_loop)

    rospy.loginfo("Success.")
    rospy.spin()

