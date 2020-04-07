#! /usr/bin/env python

import rospy
from std_msgs.msg import String, Float32, UInt8
from sensors.msg import SensorReading
from hardware.msg import DriverCommand


def on_shutdown():

    dc = DriverCommand()
    dc.header.stamp = rospy.Time.now()
    dc.source = name
    dc.priority = admin_priority
    dc.command = dc.RELEASE
    solenoid_cmd.publish(dc)

    rospy.loginfo("Exiting.")


def toggle_fill(state):

    global fill_ongoing
    fill_ongoing = state
    set_solenoid(state)


def print_relevant_data(event):

    if fill_ongoing:
        rospy.loginfo("Current pressure: {} psig; LLS: {}".format(last_pressure, last_lls))


def get_pressure(message):

    global last_pressure
    global fill_ongoing
    last_pressure = message.reading

    if message.reading > max_safe_pressure and fill_ongoing:
        rospy.logwarn("Maximum safe pressure exceeded -- stopping fill!")
        toggle_fill(False)


def get_lls_reading(message):

    global last_lls
    global fill_ongoing
    last_lls = message.reading

    if message.reading and fill_ongoing:
        rospy.logwarn("Liquid level switch tripped -- stopping fill!")
        toggle_fill(False)


def get_los(message):

    seconds = message.data
    if seconds and fill_ongoing:
        rospy.logwarn("LOS detected -- stopping fill!")
        toggle_fill(False)


def get_readiness(message):

    global current_readiness
    current_readiness = message.data


def set_solenoid(state):

    dc = DriverCommand()
    dc.header.stamp = rospy.Time.now()
    dc.source = name
    dc.priority = admin_priority
    if state:
        dc.command = dc.SOLENOID_ACTIVE
    else:
        dc.command = dc.SOLENOID_INACTIVE
    solenoid_cmd.publish(dc)


def get_command(message):

    global fill_ongoing
    command = message.data

    if command == "begin fill":
        if current_readiness is None:
            rospy.logwarn("Haven't received readiness update -- " + \
                "unsafe to continue.")
            return
        elif current_readiness != 2:
            rospy.logwarn("Fill only enabled for readiness level 2.")
            return
        rospy.logwarn("Beginning nitrous oxide fill.")
        toggle_fill(True)       
 
    elif command == "end fill":
        rospy.logwarn("Ending nitrous oxide fill.")
        toggle_fill(False)


if __name__ == "__main__":

    last_lls = None
    last_pressure = None
    fill_ongoing = False
    max_safe_pressure = 900
    admin_priority = 4
    current_readiness = None

    rospy.init_node("fill_admin", log_level=rospy.DEBUG)
    name = rospy.get_name()

    rospy.Subscriber("/commands", String, get_command)
    rospy.Subscriber("/los", Float32, get_los)
    rospy.Subscriber("/readiness_level", UInt8, get_readiness)
    rospy.Subscriber("/sensors/float_switch", SensorReading, get_lls_reading)
    rospy.Subscriber("/sensors/ox_tank_transducer", SensorReading, get_pressure)
    solenoid_cmd = rospy.Publisher("/hardware/solenoid", DriverCommand, queue_size=10)

    rospy.sleep(1)

    rospy.loginfo("Assuming control of the solenoid.")
    set_solenoid(False)

    rospy.on_shutdown(on_shutdown)

    rospy.Timer(rospy.Duration(3), print_relevant_data)

    rospy.spin()

