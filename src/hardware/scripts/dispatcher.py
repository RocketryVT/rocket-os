#!/usr/bin/env python
import rospy
from std_msgs.msg import UInt8, String, Empty
from hardware.msg import DriverCommand


def recieve_command(msg):

    command = msg.data

    driver_cmd = DriverCommand()
    driver_cmd.header.stamp = rospy.Time.now()
    driver_cmd.source = name
    driver_cmd.priority = node_priority

    if command == "all stop":
        rospy.logwarn("NOT IMPLEMENTED")

    if command == "release all":
        rospy.logwarn("NOT IMPLEMENTED")

    if command == "disable solenoid":
        rospy.logwarn("NOT IMPLEMENTED")

    elif command == "enable solenoid":
        rospy.logwarn("NOT IMPLEMENTED")

    elif command == "stop injection valve":
        rospy.logwarn("NOT IMPLEMENTED")

    elif command == "close injection valve":
        rospy.logwarn("NOT IMPLEMENTED")

    elif command == "open injection valve":
        rospy.logwarn("NOT IMPLEMENTED")

    elif command == "stop linear actuator":
        rospy.logwarn("NOT IMPLEMENTED")

    elif command == "retract linear actuator":
        rospy.logwarn("NOT IMPLEMENTED")

    elif command == "extend linear actuator":
        rospy.logwarn("NOT IMPLEMENTED")

    # ABORT VALVE ===============================

    elif command == "release abort valve":
        driver_cmd.command = driver_cmd.RELEASE
        abort_cmdr.publish(driver_cmd)

    elif command == "stop abort valve":
        driver_cmd.command = driver_cmd.STOP
        abort_cmdr.publish(driver_cmd)

    elif command == "close abort valve":
        driver_cmd.command = driver_cmd.CLOSE
        abort_cmdr.publish(driver_cmd)

    elif command == "open abort valve":
        driver_cmd.command = driver_cmd.OPEN
        abort_cmdr.publish(driver_cmd)

    elif command == "crack abort valve":
        driver_cmd.command = driver_cmd.PULSE_OPEN
        driver_cmd.pulse = rospy.Duration(3.21)
        abort_cmdr.publish(driver_cmd)

    # EMATCH=====================================

    elif command == "fire ematch":
        rospy.logwarn("NOT IMPLEMENTED")


if __name__ == "__main__":

    rospy.init_node('dispatcher', log_level=rospy.DEBUG)
    name = rospy.get_name()
    node_priority = 0

    solenoid_cmdr = rospy.Publisher("/hardware/solenoid/request", UInt8, queue_size=10)
    injection_cmdr = rospy.Publisher("/hardware/injection_valve", DriverCommand, queue_size=10)
    linear_cmdr = rospy.Publisher("/hardware/linear_actuator", DriverCommand, queue_size=10)
    abort_cmdr = rospy.Publisher("/hardware/abort_valve", DriverCommand, queue_size=10)
    ematch_cmdr = rospy.Publisher("/hardware/ematch/command", Empty, queue_size=10)

    rospy.Subscriber("/commands", String, recieve_command)
    rospy.spin()

