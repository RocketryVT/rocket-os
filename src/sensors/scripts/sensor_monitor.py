#! /usr/bin/env python

import rospy
from std_msgs.msg import String, Bool, Float32
import numpy
import re

sensor_timer = None
voltage_timer = None

class Tracker:

    def __init__(self, name, unit=None):
        self.name = name
        self.unit = unit
        self.last = None

    def get(self, message):

        self.last = message.data

    def to_string(self):

        acronym = ''.join([x[0] for x in self.name.split()]).upper()
        if self.last is None:
            return "{}: {}".format(acronym, self.last)
        if self.unit is None:
            return "{}: {:0.2f}".format(acronym, self.last)
        return "{}: {:0.2f} {}".format(acronym, self.last, self.unit)


def echo_sensors(event=None):

    echo_list_of_trackers(sensor_trackers)


def echo_voltage(event=None):

    echo_list_of_trackers(voltage_trackers)


def echo_list_of_trackers(trackers):

    output = ""
    for i, tracker in enumerate(trackers):
        output += tracker.to_string()
        if i < len(trackers) - 1:
            output = output + " / "
    rospy.loginfo(output)


def get_command(message):

    global sensor_timer
    global voltage_timer

    command = message.data

    if command == "read data":
        echo_sensors()

    elif bool(re.match(re.compile("^read data [0-9]+"), command)):

        period = float(command.split()[2])
        if sensor_timer:
            sensor_timer.shutdown()
        sensor_timer = rospy.Timer(rospy.Duration(period), echo_sensors)

    elif command == "stop data":

        if sensor_timer:
            sensor_timer.shutdown()
        sensor_timer = None

    elif command == "read voltage":
        echo_voltage()

    elif bool(re.match(re.compile("^read voltage [0-9]+"), command)):

        period = float(command.split()[2])
        if voltage_timer:
            voltage_timer.shutdown()
        voltage_timer = rospy.Timer(rospy.Duration(period), echo_voltage)

    elif command == "stop voltage":

        if voltage_timer:
            voltage_timer.shutdown()
        voltage_timer = None


rospy.init_node("sensor_monitor")

rospy.Subscriber("/commands", String, get_command)

sensor_trackers = [
    Tracker("Oxidizer tank pressure",           "psig"),
    Tracker("Combustion chamber pressure",      "psig"),
    Tracker("Oxidizer tank temperature",        "F"),
    Tracker("Combustion chamber temperature 1", "F"),
    Tracker("Combustion chamber temperature 2", "F"),
    Tracker("Float switch")
]

voltage_trackers = [
    Tracker("Oxidizer tank pressure",           "V"),
    Tracker("Combustion chamber pressure",      "V"),
    Tracker("Oxidizer tank temperature",        "V"),
    Tracker("Combustion chamber temperature 1", "V"),
    Tracker("Combustion chamber temperature 2", "V"),
    Tracker("Float switch",                     "V")
]

rospy.Subscriber("/sensors/ox_tank_transducer/pressure", Float32, sensor_trackers[0].get)
rospy.Subscriber("/sensors/combustion_transducer/pressure", Float32, sensor_trackers[1].get)
rospy.Subscriber("/sensors/ox_tank_thermocouple/temperature", Float32, sensor_trackers[2].get)
rospy.Subscriber("/sensors/combustion_thermocouple_1/temperature", Float32, sensor_trackers[3].get)
rospy.Subscriber("/sensors/combustion_thermocouple_2/temperature", Float32, sensor_trackers[4].get)
rospy.Subscriber("/sensors/float_switch/state", Bool, sensor_trackers[5].get)

rospy.Subscriber("/sensors/ox_tank_transducer/voltage", Float32, voltage_trackers[0].get)
rospy.Subscriber("/sensors/combustion_transducer/voltage", Float32, voltage_trackers[1].get)
rospy.Subscriber("/sensors/ox_tank_thermocouple/voltage", Float32, voltage_trackers[2].get)
rospy.Subscriber("/sensors/combustion_thermocouple_1/voltage", Float32, voltage_trackers[3].get)
rospy.Subscriber("/sensors/combustion_thermocouple_2/voltage", Float32, voltage_trackers[4].get)
rospy.Subscriber("/sensors/float_switch/voltage", Float32, voltage_trackers[5].get)

rospy.spin()

