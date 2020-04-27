#! /usr/bin/env python

# ematch_driver.py

import rospy
from std_msgs.msg import Empty
import time
import signal
import sys
import random
from hardware.msg import DriverCommand
import driverlib

try:
    import Adafruit_BBIO.GPIO as gpio
except:
    print("Failed to import Adafruit_BBIO.GPIO, running in desktop mode")
    gpio = None


def signal_handler(sig, frame):
    if gpio:
        gpio.cleanup()
    exit(0)


def execute_ematch_command(msg):

    global locked

    if msg.command is msg.EMATCH_LOCK:
        rospy.loginfo("Ematch locked by " + msg.source + \
            ", with priority " + str(msg.priority) + ".")
        locked = True
    elif msg.command is msg.EMATCH_UNLOCK:
        rospy.logwarn("Ematch unlocked by " + msg.source + ".")
        locked = False
        driverlib.nullify_command(msg)
    elif msg.command is msg.EMATCH_FIRE and not locked:
        rospy.loginfo("Firing e-match.")
        if gpio:
            gpio.output(ctrl_pin_a, gpio.HIGH)
        time.sleep(delay)
        if gpio:
            gpio.output(ctrl_pin_b, gpio.HIGH)
        time.sleep(2)
        if gpio:
            gpio.output(ctrl_pin_a, gpio.LOW)
            gpio.output(ctrl_pin_b, gpio.LOW)
    elif msg.command is msg.EMATCH_FIRE:
        rospy.logwarn(msg.source + " is attempting to issue a fire " + \
            "command -- the ematch has been locked. Denied.")
        driverlib.nullify_command(msg)
    else:
        rospy.logwarn("Unimplemented command: " + str(msg.command))
        driverlib.nullify_command(msg)


if __name__ == "__main__":

    locked = True

    signal.signal(signal.SIGINT, signal_handler)

    if gpio:
        gpio.cleanup()

    rospy.init_node("ematch_driver", log_level=rospy.DEBUG);
    name = rospy.get_name()

    try:
        ctrl_pin_a = rospy.get_param(name + "/pin_a")
        ctrl_pin_b = rospy.get_param(name + "/pin_b")
        delay = rospy.get_param(name + "/delay")
    except:
        rospy.logerr("Failed to retrieve configuration from rosparam server.")
        rospy.signal_shutdown("Unavailable config.")
        exit()
    rospy.loginfo("Starting e-match driver on pins " + \
        ctrl_pin_a + ", " + ctrl_pin_b)
    rospy.loginfo("Using a delay of {} seconds.".format(delay))

    success = False
    max_attempts = 10
    for i in range(max_attempts):

        try:
            if gpio:
                gpio.setup(ctrl_pin_a, gpio.OUT)
                gpio.setup(ctrl_pin_b, gpio.OUT)
            success = True
        except:
            sleep = random.randint(1, 20)
            rospy.logwarn(str(i) + ": Failed to configure. Waiting for " \
                + str(sleep) + " seconds before reattempt.")
            rospy.sleep(sleep)

        if success:
            break

    if not success:
        rospy.logerr("Failed to configure after " + \
            str(max_attempts) + " attempts.")
        exit()

    if gpio:
        gpio.output(ctrl_pin_a, gpio.LOW)
        gpio.output(ctrl_pin_b, gpio.LOW)

    driverlib.callback(execute_ematch_command)
    rospy.Subscriber(name, DriverCommand, driverlib.receive_command);
    rospy.loginfo("Success.")
    rospy.spin()

