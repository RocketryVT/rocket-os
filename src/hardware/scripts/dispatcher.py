#!/usr/bin/env python
import rospy
from std_msgs.msg import UInt8, String, Bool, Empty

def recieve_command(msg):

    command = msg.data.rstrip()

    if command == "close solenoid":
        solenoid_cmdr.publish(False)

    elif command == "open solenoid":
        solenoid_cmdr.publish(True)

    elif command == "close ignition valve":
        ignition_cmdr.publish(1)

    elif command == "open ignition valve":
        ignition_cmdr.publish(2)

    elif command == "retract linear actuator":
        linear_cmdr.publish(1)

    elif command == "extend linear actuator":
        linear_cmdr.publish(2)

    elif command == "close vent valve":
        vent_cmdr.publish(1)

    elif command == "open vent valve":
        vent_cmdr.publish(2)

    elif command == "fire ematch":
        ematch_cmdr.publish()

rospy.init_node('dispatcher')

solenoid_cmdr = rospy.Publisher("/hardware/solenoid/command", Bool, queue_size=10)
ignition_cmdr = rospy.Publisher("/hardware/ignition_valve/command", UInt8, queue_size=10)
linear_cmdr = rospy.Publisher("/hardware/linear_actuator/command", UInt8, queue_size=10)
ematch_cmdr = rospy.Publisher("/hardware/ematch/command", Empty, queue_size=10)
vent_cmdr = rospy.Publisher("/hardware/vent_valve/command", UInt8, queue_size=10)

rospy.Subscriber("/commands", String, recieve_command)
rospy.spin()
