#! /usr/bin/env python

import rospy
from std_msgs.msg import String, UInt8, Bool, Float32

last_pressure = None
last_lls = None
fill_ongoing = False

def print_relevant_data(event):

    if last_pressure is not None and last_lls is not None and fill_ongoing:
        rospy.loginfo("Current pressure: {:0.2f} psig; LLS: {}".format(last_pressure, last_lls))

def get_pressure(message):

    global last_pressure
    last_pressure = message.data

def get_lls_reading(message):

    global last_lls
    global fill_ongoing
    last_lls = message.data

    if message.data:
        rospy.logwarn("Liquid level switch tripped -- stopping fill!")
        fill_ongoing = False

def get_command(message):

    global fill_ongoing
    command = message.data

    if command == "begin fill":
        rospy.logwarn("Beginning nitrous oxide fill.")
        fill_ongoing = True

    elif command == "end fill":
        rospy.logwarn("Ending nitrous oxide fill.")
        fill_ongoing = False


rospy.init_node("fill_admin")

rospy.Subscriber("/commands", String, get_command)
rospy.Subscriber("/sensors/float_switch/state", Bool, get_lls_reading)
rospy.Subscriber("/sensors/ox_tank_transducer/pressure", Float32, get_pressure)

rospy.Timer(rospy.Duration(10), print_relevant_data)

rospy.spin()
