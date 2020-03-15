#! /usr/bin/env python

import rospy
from std_msgs.msg import String, Bool, UInt8, Float32

last_pressure = None
last_lls = None
fill_ongoing = False
max_safe_pressure = 900 # psig

def toggle_fill(state):

    global fill_ongoing
    fill_ongoing = state
    solenoid_cmd.publish(state)

def print_relevant_data(event):

    if fill_ongoing:
        rospy.loginfo("Current pressure: {} psig; LLS: {}".format(last_pressure, last_lls))

def get_pressure(message):

    global last_pressure
    global fill_ongoing
    last_pressure = message.data

    if message.data > max_safe_pressure and fill_ongoing:
        rospy.logwarn("Maximum safe pressure exceeded -- stopping fill!")
        toggle_fill(False)

def get_lls_reading(message):

    global last_lls
    global fill_ongoing
    last_lls = message.data

    if message.data and fill_ongoing:
        rospy.logwarn("Liquid level switch tripped -- stopping fill!")
        toggle_fill(False)

def get_los(message):

    seconds = message.data
    if seconds and fill_ongoing:
        rospy.logwarn("LOS detected -- stopping fill!")
        toggle_fill(False)

def get_readiness(message):

    readiness = message.data
    if readiness != 2 and fill_ongoing:
        rospy.logwarn("Fill only enabled for readiness level 2.")
        toggle_fill(False)

def get_command(message):

    global fill_ongoing
    command = message.data

    if command == "begin fill":
        rospy.logwarn("Beginning nitrous oxide fill.")
        toggle_fill(True)       
 
    elif command == "end fill":
        rospy.logwarn("Ending nitrous oxide fill.")
        toggle_fill(False)


rospy.init_node("fill_admin")

rospy.Subscriber("/commands", String, get_command)
rospy.Subscriber("/los", Float32, get_los)
rospy.Subscriber("/readiness_level", UInt8, get_readiness)
rospy.Subscriber("/sensors/float_switch/state", Bool, get_lls_reading)
rospy.Subscriber("/sensors/ox_tank_transducer/pressure", Float32, get_pressure)
solenoid_cmd = rospy.Publisher("/hardware/solenoid/request", UInt8, queue_size=10)

rospy.sleep(1)

rospy.loginfo("Assuming primary control of the solenoid.")
solenoid_cmd.publish(0)

rospy.Timer(rospy.Duration(3), print_relevant_data)

rospy.spin()

