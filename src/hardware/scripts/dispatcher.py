#!/usr/bin/env python
import rospy
from std_msgs.msg import UInt8, String, Bool

def recieve_command(msg):

    global solenoid_cmdr
    global linear_cmdr
    global ignition_cmdr

    command = msg.data.rstrip()

    print(command, len(command))

    if command == "close solenoid":
        rospy.loginfo("Got " + command)
        solenoid_cmdr.publish(False)

    elif command == "open solenoid":
        rospy.loginfo("Got " + command)
        solenoid_cmdr.publish(True)

    elif command == "close ignition valve":
        rospy.loginfo("Got " + command)
        ignition_cmdr.publish(1)

    elif command == "open ignition valve":
        rospy.loginfo("Got " + command)
        ignition_cmdr.publish(2)

    elif command == "retract linear actuator":
        rospy.loginfo("Got " + command)
        linear_cmdr.publish(1)

    elif command == "extend linear actuator":
        rospy.loginfo("Got " + command)
        linear_cmdr.publish(2)


rospy.init_node('dispatcher')

solenoid_cmdr = rospy.Publisher("/solenoid_command", Bool, queue_size=10)
ignition_cmdr = rospy.Publisher("/ignition_valve_command", UInt8, queue_size=10)
linear_cmdr = rospy.Publisher("/linear_actuator_command", UInt8, queue_size=10)

rospy.Subscriber("/commands", String, recieve_command)
rospy.spin()
