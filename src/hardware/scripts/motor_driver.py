#! /usr/bin/env python3

# motor_driver.py

'''
Motor: A Generic Motor Driver

+ Num of Functions: 1

'''

import rospy
from hardware.msg import DriverCommand
import sys
import random
import yaml
import driverlib

try:
import Adafruit_BBIO.GPIO as gpio
except:
gpio = None


def execute_motor_command(msg):
    ''' Controls the Motor based on the given commands.
    + Stop Motor
    + Close Motor
    + Open Motor
    + Pulse Close
    + Pulse Open

    @param msg: A DriverCommand message
    '''
    rospy.logdebug("Executing new command: " + driverlib.cmd2str(msg))

    # Clockwise Pin: cw_pin
    # Counter Clockwise Pin: ccw_pin

    # If Stopping Motor: 
    if msg.command is msg.MOTOR_STOP:
        rospy.loginfo("Command from " + msg.source + ": stop the motor")
    if gpio:
        gpio.output(cw_pin, gpio.LOW)
        gpio.output(ccw_pin, gpio.LOW)

    # If Closing Motor: 
    elif msg.command is msg.MOTOR_CLOSE:
        rospy.loginfo("Command from " + msg.source + ": close the motor")
    if gpio:
        gpio.output(cw_pin, gpio.HIGH)
        gpio.output(ccw_pin, gpio.LOW)

    # If Opening Motor: 
    elif msg.command is msg.MOTOR_OPEN:
        rospy.loginfo("Command from " + msg.source + ": open the motor")
    if gpio:
        gpio.output(cw_pin, gpio.LOW)
        gpio.output(ccw_pin, gpio.HIGH)

    # If Closing Motor For (?sec) Then Stopping Motor: 
    elif msg.command is msg.MOTOR_PULSE_CLOSE:
        rospy.loginfo("Command from " + msg.source + ": pulse-close " +
        "the motor for {} seconds".format(msg.pulse.to_sec()))
        rospy.loginfo("Closing...")
        now = rospy.Time.now()
    if gpio:
        gpio.output(cw_pin, gpio.HIGH)
        gpio.output(ccw_pin, gpio.LOW)
        rospy.sleep(msg.pulse)
    if gpio:
        gpio.output(cw_pin, gpio.LOW)
        gpio.output(ccw_pin, gpio.LOW)
        elapsed = rospy.Time.now() - now
        rospy.loginfo("Stopped. {} seconds elapsed.".format(elapsed.to_sec()))

    # If Opening Motor For (?sec) Then Stopping Motor: 
    elif msg.command is msg.MOTOR_PULSE_OPEN:
        dur = msg.pulse.to_sec()
        rospy.loginfo("Command from " + msg.source + ": pulse-open " +
        "the motor for {} seconds".format(msg.pulse.to_sec()))
        rospy.loginfo("Opening...")
        now = rospy.Time.now()
    if gpio:
        gpio.output(cw_pin, gpio.LOW)
        gpio.output(ccw_pin, gpio.HIGH)
        rospy.sleep(msg.pulse)
    if gpio:
        gpio.output(cw_pin, gpio.LOW)
        gpio.output(ccw_pin, gpio.LOW)
        elapsed = rospy.Time.now() - now
        rospy.loginfo("Stopped. {} seconds elapsed.".format(elapsed.to_sec()))

    # If Some other Commmand: Warn that command did nothing.
    else:
        rospy.logwarn("Unimplemented command: " + str(msg.command))
        driverlib.nullify_command(msg) 




if __name__ == "__main__":

    # Initialize Node
    rospy.init_node("motor_driver", log_level=rospy.DEBUG);
    name = rospy.get_name()

    try:
        # Returns Values From Parameter Server
        cw_pin = rospy.get_param(name + "/pin_a") # - Get Clockwise Pin Num
        ccw_pin = rospy.get_param(name + "/pin_b") # - Get Counter Clockwise Pin Num
    except:
        rospy.logerr("Failed to retrieve pin config from rosparam server.")
        rospy.signal_shutdown("Unavailable pin config.")
        exit()

    if not gpio:
        rospy.logwarn("Failed to import Adafruit_BBIO.gpio, running in desktop mode")

    rospy.loginfo("Starting DC motor driver on pins " + cw_pin + ", " + ccw_pin)

    success = False
    max_attempts = 10
    # Try ([max_attempts] times at most) to set the 2 Motor Pins to Output
    for i in range(max_attempts):
        try:
            if gpio:
                gpio.setup(cw_pin, gpio.OUT)
                gpio.setup(ccw_pin, gpio.OUT)
            success = True
        except:
            sleep = random.randint(1, 20)
            rospy.logwarn(str(i) + ": Failed to configure. Waiting for " + \
            str(sleep) + " seconds before reattempt.")

        if success:
            break

    # If Failure to Configure Pins: Log Diagnostics and EXIT.
    if not success:
        rospy.logerr("Failed to configure after " + \
        str(max_attempts) + " attempts.")
        exit()

    # Set Newly Configured Output Pins to LOW
    if gpio:
        gpio.output(cw_pin, gpio.LOW)
        gpio.output(ccw_pin, gpio.LOW)

    # ???????????????????????????????????????? 
    driverlib.callback(execute_motor_command)

    # Set Subscription. 
    # (Send incomming messages to driverlib.receive_commands)
    rospy.Subscriber(name, DriverCommand, driverlib.receive_command);
    rospy.loginfo("Success.")
    rospy.spin()

