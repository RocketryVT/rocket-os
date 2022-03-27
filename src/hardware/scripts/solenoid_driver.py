#! /usr/bin/env python3

# solenoid_driver.py

'''
Solenoid Valve:
			Solenoid: An electromagnet

			Turning Solenoid On/Off will Open/Close a valve
			(through use of an electromagnetic field)
			Valves usually used for controlling the flow of fluid in a system.

		  + Num of Functions: 3
'''

import rospy
import sys
import time
import signal
import random
from hardware.msg import DriverCommand
import driverlib

# IDEAS:
# Declare globals once at top of file?

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


def execute_solenoid_command(msg):

	''' Activates or Deactivates the Solenoid based on the given command.

		@param msg: A DriverCommand message
	'''

    global nominal_state
	
	# If Activating Solenoid: Enable Solenoid's Open Cycle
    if msg.command == msg.SOLENOID_ACTIVE:
        rospy.loginfo("Enable solenoid open cycle")
        nominal_state = True
		
	# If Deactivating Solenoid: Disable Solenoid's Open Cycle
    elif msg.command == msg.SOLENOID_INACTIVE:
        rospy.loginfo("Disable solenoid open cycle")
        nominal_state = False
    
	# If Some other Commmand: Warn that command did nothing.
	else:
        rospy.logwarn("Unimplemented command: " + str(msg.command))
        driverlib.nullify_command(msg)


def control_loop(event):

	''' 
		Opens the Solenoid Valve on at :00 seconds or :30 seconds, 
		provided the vehicle is in a Nominal State.
		
		Else, the Solenoid Valve is closed.
		
		@param event: Not used
	'''
    global nominal_state #Boolean
    global current_state #Boolean

	# Standard Unix Time % 60 sec (Solenoid Only Activates on the :00sec or the :30sec)
    secs = rospy.Time.now().to_sec() % (opened_secs + closed_secs)

	# If sec < 30 & Nominal: Open Solenoid
    if (nominal_state and secs < opened_secs) and not current_state:
        rospy.loginfo("Opening the solenoid")
        current_state = True
        # Set ctrl_pin HIGH
		if gpio:
            gpio.output(ctrl_pin, gpio.HIGH)

	# Else: Close Solenoid
    elif not (nominal_state and secs < opened_secs) and current_state:
        rospy.loginfo("Closing the solenoid")
        current_state = False
        # Set ctrl_pin LOW
		if gpio:
            gpio.output(ctrl_pin, gpio.LOW) 


if __name__ == "__main__":

    nominal_state = False
    current_state = False

	# Reset All Pins You've Used to INPUT
    if gpio:
        gpio.cleanup()

	# Initialize Node
    rospy.init_node("solenoid_driver", log_level=rospy.DEBUG)

	name = rospy.get_name() #node name

    if not gpio:
        rospy.logwarn("Failed to import Adafruit_BBIO.gpio, running in desktop mode")

    try:
		# Returns Values From Parameter Server 
        ctrl_pin = rospy.get_param(name + "/pin") # - Returns Ctrl Pin Num
        opened_secs = rospy.get_param(name + "/opened") # - Get Time Solenoid stays opened (sec)
        closed_secs = rospy.get_param(name + "/closed") # - Get Time Solenoid stays closed (sec)
    except:
        rospy.logerr("Failed to retrieve configuration from rosparam server.")
        rospy.signal_shutdown("Unavailable config.")
        exit()
    
	rospy.loginfo("Starting solenoid driver on pin " + ctrl_pin)
    rospy.loginfo("Using {} s, {} s open-close cycle.".format( \
        opened_secs, closed_secs))

    success = False
    max_attempts = 10
	# Try ([max_attempts] times at most) to set ctrl_pin as the Solenoid's Output Pin
    for i in range(max_attempts):

        try:
            if gpio:
                gpio.setup(ctrl_pin, gpio.OUT)
            success = True
        except:
            sleep = random.randint(1, 20)
            rospy.logwarn(str(i) + ": Failed to configure. " + \
                "Waiting for " + str(sleep) + " seconds before reattempt.")
            rospy.sleep(sleep)

        if success:
            break

	# If Failure to Configure Pin: Log Diagnostics and EXIT.
    if not success:
        rospy.logerr("Failed to configure after " + \
            str(max_attempts) + " attempts.")
        exit()

	# Set Newly Configured Solenoid Output Pin to LOW
    if gpio:
        gpio.output(ctrl_pin, gpio.LOW)

	# ???????????????????????????????????????? 
    driverlib.callback(execute_solenoid_command) 
    
	# Set Subscription. 
	# (Send incomming messages to driverlib.receive_commands)
	rospy.Subscriber(name, DriverCommand, driverlib.receive_command);
    
	# Frequency that messages are published to the Topic
	rospy.Timer(rospy.Duration(0.5), control_loop)

    rospy.loginfo("Success.")
    rospy.spin()

