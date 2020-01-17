#! /usr/bin/env python

# solenoid_driver.py

import rospy
from std_msgs.msg import Bool

nominal_state = False
current_state = False

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
    secs = rospy.Time.now().secs % 60

    if (nominal_state and secs < 28) and not current_state:
        rospy.loginfo("Opening the solenoid")
        current_state = True
    elif not (nominal_state and secs < 2) and current_state:
        rospy.loginfo("Closing the solenoid")
        current_state = False

rospy.init_node("solenoid_driver");

rospy.Subscriber("command", Bool, recieve_command);

rospy.Timer(rospy.Duration(0.5), control_loop)

rospy.spin()

