#! /usr/bin/env python

import rospy
from std_msgs.msg import String, UInt8, Bool, Float32
import numpy
import re

suppress_warnings = False
suppression_reminder_period = 60 # seconds
moving_average_period = 500; # readings, not seconds
warning_period = 10 # min seconds between warnings
error_period = 5 # min seconds between avg warnings
nitrous_tank_abort_threshold_psi = 1200
echo_timer = None

def throw_suppression_reminder():

    rospy.logwarn_throttle(suppression_reminder_period, "Warnings are being suppressed.")

class SensorTracker:

    def __init__(self, name, unit, limits, ctrl_vent_valve=False):
        self.name = name
        self.limits = limits
        self.unit = unit
        self.history = []
        self.ctrl_vent_valve = ctrl_vent_valve

    def get(self, message):
        self.history.append(message.data)
        while len(self.history) > moving_average_period:
            self.history.pop(0)
        avg = numpy.mean(self.history)

        (min, max) = self.limits
        warning = self.name + " {} ({:.2f}) falls outside of nominal bounds (" + str(self.limits) + ")"

        if message.data < min or message.data > max:
            if not suppress_warnings:
                rospy.logwarn_throttle(warning_period, warning.format("reading", message.data))
            else:
                throw_suppression_reminder()

        if avg < min or avg > max:
            if not suppress_warnings:
                rospy.logerr_throttle(error_period,
                    warning.format("average", avg))
            else:
                throw_suppression_reminder()

        if self.ctrl_vent_valve and avg > nitrous_tank_abort_threshold_psi:
            overpressure_detected(message.data, avg)

    def last_reading(self):

        if self.history:
            return self.history[-1]
        return None


def overpressure_detected(current, avg):

    rospy.logwarn_throttle(error_period, "Nitrous tank overpressure ({}, {} >1200 psig) detected!".format(current, avg))


def echo_sensors(event):

    output = ""
    for i in range(0, len(trackers)):
        tracker = trackers[i]
        acronym = ''.join([x[0] for x in tracker.name.split()]).upper()
        last = tracker.last_reading()
        if last is None:
            last = 'None'
        else:
            last = "{:.2f}".format(last)
        output = output + acronym + ": " + last + " " + tracker.unit
        if i < len(trackers) - 1:
            output = output + " / "
    rospy.loginfo(output)
            

def get_command(message):

    global echo_timer
    global suppress_warnings

    command = message.data
    if command == "read data":
        echo_sensors(None)

    elif bool(re.match(re.compile("^read data [0-9]+"), command)):

        period = float(command.split()[2])
        if echo_timer:
            echo_timer.shutdown()
        echo_timer = rospy.Timer(rospy.Duration(period), echo_sensors)

    elif command == "stop data":

        if echo_timer:
            echo_timer.shutdown()
        echo_timer = None

    elif command == "toggle sensor warning suppression":

        suppress_warnings = not suppress_warnings
        rospy.loginfo("Sensor warning suppression set to " + str(suppress_warnings))


rospy.init_node("sensor_monitor")

rospy.Subscriber("/commands", String, get_command)

trackers = [
    SensorTracker("Oxidizer tank pressure",            "psig",  [0, 900], True),
    SensorTracker("Combustion chamber pressure",       "psig",  [0, 50]),
    SensorTracker("Oxidizer tank temperature",         "F",     [0, 95]),
    SensorTracker("Combustion chamber temperature 1", "F",     [0, 95]),
    SensorTracker("Combustion chamber temperature 2", "F",     [0, 95])
]

rospy.Subscriber("/sensors/ox_tank_transducer/pressure", Float32, trackers[0].get)
rospy.Subscriber("/sensors/combustion_transducer/pressure", Float32, trackers[1].get)
rospy.Subscriber("/sensors/ox_tank_thermocouple/temperature", Float32, trackers[2].get)
rospy.Subscriber("/sensors/combustion_thermocouple_1/temperature", Float32, trackers[3].get)
rospy.Subscriber("/sensors/combustion_thermocouple_2/temperature", Float32, trackers[4].get)

rospy.spin()
