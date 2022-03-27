#! /usr/bin/env python3

# ematch_driver.py

'''
E-Match: An electronic fireworks igniter.
         They're pretty cool.

         + Num of Functions: 2
'''

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
    gpio = None


def signal_handler(sig, frame):
    ''' Resets All Pins You've Used to INPUT.
        Then Exits Program.

        @param sig: Not used
        @param frame: Not used
    '''

    if gpio:
        gpio.cleanup()
    exit(0)


def execute_ematch_command(msg):
    ''' Controls the E-Match based on the given commands.
            + Command to Lock E-Match
            + Command to Unlock E-Match
            + Command to Fire E-Match

        @param msg: A DriverCommand message
    '''

    global locked

    # If Locking E-Match: Set locked variable to True
    if msg.command is msg.EMATCH_LOCK:
        rospy.loginfo("Ematch locked by " + msg.source +
                      ", with priority " + str(msg.priority) + ".")
        locked = True

    # If Unlocking E-Match: Set locked variable to False
    elif msg.command is msg.EMATCH_UNLOCK:
        rospy.logwarn("Ematch unlocked by " + msg.source + ".")
        locked = False

        # ?????????????????????????????????????????
        driverlib.nullify_command(msg)

    # If Firing while E-Match is Unlocked:
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

    # If Firing while E-Match is Locked:
    elif msg.command is msg.EMATCH_FIRE:
        rospy.logwarn(msg.source + " is attempting to issue a fire " +
                      "command -- the ematch has been locked. Denied.")
        driverlib.nullify_command(msg)

    # If Some other Commmand: Warn that command did nothing.
    else:
        rospy.logwarn("Unimplemented command: " + str(msg.command))
        driverlib.nullify_command(msg)


if __name__ == "__main__":

    locked = True

    signal.signal(signal.SIGINT, signal_handler)

    if gpio:
        gpio.cleanup()
    else:
        rospy.logwarn(
            "Failed to import Adafruit_BBIO.gpio, running in desktop mode")

    # Initialize Node
    rospy.init_node("ematch_driver", log_level=rospy.DEBUG)
    name = rospy.get_name()

    try:
        # Returns Values From Parameter Server
        ctrl_pin_a = rospy.get_param(
            name + "/pin_a")  # - Get Control Pin A Num
        ctrl_pin_b = rospy.get_param(
            name + "/pin_b")  # - Get Control Pin B Num
        delay = rospy.get_param(name + "/delay")  # - Get E-Match Delay amount
    except:
        rospy.logerr("Failed to retrieve configuration from rosparam server.")
        rospy.signal_shutdown("Unavailable config.")
        exit()
    rospy.loginfo("Starting e-match driver on pins " +
                  ctrl_pin_a + ", " + ctrl_pin_b)
    rospy.loginfo("Using a delay of {} seconds.".format(delay))

    success = False
    max_attempts = 10
    # Try ([max_attempts] times at most) to set the 2 E-Match Pins to Output
    for i in range(max_attempts):
        try:
            if gpio:
                gpio.setup(ctrl_pin_a, gpio.OUT)
                gpio.setup(ctrl_pin_b, gpio.OUT)
            success = True
        except:
            sleep = random.randint(1, 20)
            rospy.logwarn(str(i) + ": Failed to configure. Waiting for "
                          + str(sleep) + " seconds before reattempt.")
            rospy.sleep(sleep)
        if success:
            break

    # If Failure to Configure Pins: Log Diagnostics and EXIT.
    if not success:
        rospy.logerr("Failed to configure after " +
                     str(max_attempts) + " attempts.")
        exit()

    # Set Newly Configured Output Pins to LOW
    if gpio:
        gpio.output(ctrl_pin_a, gpio.LOW)
        gpio.output(ctrl_pin_b, gpio.LOW)

    # ????????????????????????????????????????
    driverlib.callback(execute_ematch_command)

    # Set Subscription.
    # (Send incomming messages to driverlib.receive_commands)
    rospy.Subscriber(name, DriverCommand, driverlib.receive_command)
    rospy.loginfo("Success.")
    rospy.spin()
