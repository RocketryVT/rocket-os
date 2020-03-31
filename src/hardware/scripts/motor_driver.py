#! /usr/bin/env python

# motor_driver.py

import rospy
from hardware.msg import DriverCommand
import sys
import Adafruit_BBIO.GPIO as gpio
import random
import yaml

def cmd2str(msg):

    return str(yaml.load(str(msg)))

def execute_command(msg):

    rospy.logdebug("Executing new command: " + cmd2str(msg))

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

    elif msg.command is msg.PULSE_OPEN:
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
    

def get_highest_priority_command():

    selected = None
    for cmd in all_commands.values():
        rospy.logdebug(">> " + cmd2str(cmd))
        if selected is None or cmd.priority > selected.priority:
            selected = cmd
        elif cmd.priority == selected.priority:
            if cmd.header.stamp < selected.header.stamp:
                selected = cmd

    rospy.logdebug("HPC: " + cmd2str(selected))
    return selected


def recieve_command(msg):

    rospy.logdebug("Received new command: " + cmd2str(msg))

    old_cmd = get_highest_priority_command()

    # overwrite any previous command from this source
    all_commands[msg.source] = msg
    if msg.command == msg.RELEASE:
        rospy.loginfo("Command from " + msg.source + ": release the motor")
        all_commands.pop(msg.source)

    new_cmd = get_highest_priority_command()

    if new_cmd == old_cmd:
        rospy.logdebug("HPC remains unchanged. Doing nothing.")
    elif new_cmd is not None:
        execute_command(new_cmd)


if __name__ == "__main__":

    all_commands = {}

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
        rospy.logerr("Failed to configure after " + str(max_attempts) + " attempts.")
        exit()

    gpio.output(cw_pin, gpio.LOW)
    gpio.output(ccw_pin, gpio.LOW)

    rospy.Subscriber(name, DriverCommand, recieve_command);
    rospy.loginfo("Success.")
    rospy.spin()

