#!/usr/bin/env python
import rospy
from std_msgs.msg import UInt8, String, Empty

def recieve_command(msg):

    command = msg.data

    if command == "all stop":
        solenoid_cmdr.publish(0)
        injection_cmdr.publish(0)
        linear_cmdr.publish(0)
        abort_cmdr.publish(0)

    if command == "release all":
        solenoid_cmdr.publish(255)
        injection_cmdr.publish(255)
        linear_cmdr.publish(255)
        abort_cmdr.publish(255)

    if command == "disable solenoid":
        solenoid_cmdr.publish(0)

    elif command == "enable solenoid":
        solenoid_cmdr.publish(1)

    elif command == "stop injection valve":
        injection_cmdr.publish(0)

    elif command == "close injection valve":
        injection_cmdr.publish(1)

    elif command == "open injection valve":
        injection_cmdr.publish(2)

    elif command == "stop linear actuator":
        linear_cmdr.publish(0)

    elif command == "retract linear actuator":
        linear_cmdr.publish(1)

    elif command == "extend linear actuator":
        linear_cmdr.publish(2)

    elif command == "stop abort valve":
        abort_cmdr.publish(0)

    elif command == "close abort valve":
        abort_cmdr.publish(1)

    elif command == "open abort valve":
        abort_cmdr.publish(2)

    elif command == "crack abort valve":
        abort_cmdr.publish(2)
        now = rospy.get_time()
        rospy.sleep(0.1)
        later = rospy.get_time()
        abort_cmdr.publish(0)

    elif command == "fire ematch":
        ematch_cmdr.publish()

rospy.init_node('dispatcher')

solenoid_cmdr = rospy.Publisher("/hardware/solenoid/request", UInt8, queue_size=10)
injection_cmdr = rospy.Publisher("/hardware/injection_valve/request", UInt8, queue_size=10)
linear_cmdr = rospy.Publisher("/hardware/linear_actuator/request", UInt8, queue_size=10)
abort_cmdr = rospy.Publisher("/hardware/abort_valve/request", UInt8, queue_size=10)
ematch_cmdr = rospy.Publisher("/hardware/ematch/command", Empty, queue_size=10)

rospy.Subscriber("/commands", String, recieve_command)
rospy.spin()
