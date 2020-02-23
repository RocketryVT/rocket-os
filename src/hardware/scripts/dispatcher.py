#!/usr/bin/env python
import rospy
from std_msgs.msg import UInt8, String, Bool, Empty

def recieve_command(msg):

    command = msg.data

    if command == "all stop":
        solenoid_cmdr.publish(False)
        ignition_cmdr.publish(0)
        linear_cmdr.publish(0)
        vent_cmdr.publish(0)

    if command == "close solenoid":
        solenoid_cmdr.publish(False)

    elif command == "open solenoid":
        solenoid_cmdr.publish(True)

    elif command == "stop ignition valve":
        ignition_cmdr.publish(0)

    elif command == "close ignition valve":
        ignition_cmdr.publish(1)

    elif command == "open ignition valve":
        ignition_cmdr.publish(2)

    elif command == "stop linear actuator":
        linear_cmdr.publish(0)

    elif command == "retract linear actuator":
        linear_cmdr.publish(1)

    elif command == "extend linear actuator":
        linear_cmdr.publish(2)

    elif command == "stop vent valve":
        vent_cmdr.publish(0)

    elif command == "close vent valve":
        vent_cmdr.publish(1)

    elif command == "open vent valve":
        vent_cmdr.publish(2)

    elif command == "crack vent valve":
        vent_cmdr.publish(2)
        now = rospy.get_time()
        rospy.sleep(0.1)
        later = rospy.get_time()
        vent_cmdr.publish(0)

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
