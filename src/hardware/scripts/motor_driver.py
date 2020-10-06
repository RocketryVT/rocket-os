#! /usr/bin/env python

# motor_driver.py

import rospy
from hardware.msg import DriverCommand
import sys
import Adafruit_BBIO.GPIO as gpio
import random
import yaml
import driverlib

def execute_motor_command(msg):

    rospy.logdebug("Executing new command: " + driverlib.cmd2str(msg))

    if msg.command is msg.MOTOR_STOP:
        rospy.loginfo("Command from " + msg.source + ": stop the motor")
        gpio.output(cw_pin, gpio.LOW)
        gpio.output(ccw_pin, gpio.LOW)

    elif msg.command is msg.MOTOR_CLOSE:
        rospy.loginfo("Command from " + msg.source + ": close the motor")
        gpio.output(cw_pin, gpio.HIGH)
        gpio.output(ccw_pin, gpio.LOW)

    elif msg.command is msg.MOTOR_OPEN:
        rospy.loginfo("Command from " + msg.source + ": open the motor")
        gpio.output(cw_pin, gpio.LOW)
        gpio.output(ccw_pin, gpio.HIGH)

    elif msg.command is msg.MOTOR_PULSE_CLOSE:
        rospy.loginfo("Command from " + msg.source + ": pulse-close " +
            "the motor for {} seconds".format(msg.pulse.to_sec()))
        rospy.loginfo("Closing...")
        now = rospy.Time.now()
        gpio.output(cw_pin, gpio.HIGH)
        gpio.output(ccw_pin, gpio.LOW)
        rospy.sleep(msg.pulse)
        gpio.output(cw_pin, gpio.LOW)
        gpio.output(ccw_pin, gpio.LOW)
        elapsed = rospy.Time.now() - now
        rospy.loginfo("Stopped. {} seconds elapsed.".format(elapsed.to_sec()))

    elif msg.command is msg.MOTOR_PULSE_OPEN:
        dur = msg.pulse.to_sec()
        rospy.loginfo("Command from " + msg.source + ": pulse-open " +
            "the motor for {} seconds".format(msg.pulse.to_sec()))
        rospy.loginfo("Opening...")
        now = rospy.Time.now()
        gpio.output(cw_pin, gpio.LOW)
        gpio.output(ccw_pin, gpio.HIGH)
        rospy.sleep(msg.pulse)
        gpio.output(cw_pin, gpio.LOW)
        gpio.output(ccw_pin, gpio.LOW)
        elapsed = rospy.Time.now() - now
        rospy.loginfo("Stopped. {} seconds elapsed.".format(elapsed.to_sec()))
    else:
        rospy.logwarn("Unimplemented command: " + str(msg.command))
        driverlib.nullify_command(msg) 

if __name__ == "__main__":

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
            rospy.logwarn(str(i) + ": Failed to configure. Waiting for " + \
                str(sleep) + " seconds before reattempt.")
            rospy.sleep(sleep)
        if success:
            break

    if not success:
        rospy.logerr("Failed to configure after " + \
            str(max_attempts) + " attempts.")
        exit()

    gpio.output(cw_pin, gpio.LOW)
    gpio.output(ccw_pin, gpio.LOW)

    driverlib.callback(execute_motor_command)
    rospy.Subscriber(name, DriverCommand, driverlib.receive_command);
    rospy.loginfo("Success.")
    rospy.spin()

