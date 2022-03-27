#! /usr/bin/env python3

'''
Sensor Monitor: ...

              + Num of Functions: 4
              + Num of Classes: 1
'''


import rospy
from std_msgs.msg import String, Bool, Float32
from sensors.msg import SensorReading
import numpy
import re

sensor_timer = None
voltage_timer = None


class Tracker:

    '''
        ?
    '''

    def __init__(self, name, unit=None):
        self.name = name
        self.last = None

    def get(self, message):

        self.last = message

    def to_string(self, voltage=False):

        acronym = ''.join([x[0] for x in self.name.split()]).upper()
        if self.last is None:
            return "{}:     ".format(acronym)
        if voltage:
            return "{}: {:0.2f} V".format(acronym, self.last.voltage)
        if self.last.unit == "":
            return "{}: {:0.2f}".format(acronym, self.last.reading)
        return "{}: {:0.2f} {}".format(acronym, self.last.reading, self.last.unit)


def echo_sensors(event=None):
    '''
        ?
    '''

    echo(False)


def echo_voltage(event=None):
    '''
        ?
    '''

    echo(True)


def echo(print_voltage):
    '''
        ?
    '''

    output = ""
    for i, tracker in enumerate(trackers):
        output += tracker.to_string(print_voltage)
        if i < len(trackers) - 1:
            output = output + " / "
    rospy.loginfo(output)


def get_command(message):
    '''
        ?
    '''

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


if __name__ == "__main__":

    # Initialize Node
    rospy.init_node("sensor_monitor", log_level=rospy.DEBUG)

    rospy.Subscriber("/commands", String, get_command)

    trackers = [
        Tracker("Oxidizer tank pressure"),
        Tracker("Combustion chamber pressure"),
        Tracker("Oxidizer tank temperature"),
        Tracker("Combustion chamber temperature 1"),
        Tracker("Combustion chamber temperature 2"),
        Tracker("Float switch")
    ]

    # Set Subscriptions
    rospy.Subscriber("/sensors/ox_tank_transducer",
                     SensorReading, trackers[0].get)
    rospy.Subscriber("/sensors/combustion_transducer",
                     SensorReading, trackers[1].get)
    rospy.Subscriber("/sensors/ox_tank_thermocouple",
                     SensorReading, trackers[2].get)
    rospy.Subscriber("/sensors/combustion_thermocouple_1",
                     SensorReading, trackers[3].get)
    rospy.Subscriber("/sensors/combustion_thermocouple_2",
                     SensorReading, trackers[4].get)
    rospy.Subscriber("/sensors/float_switch", SensorReading, trackers[5].get)

    rospy.spin()
