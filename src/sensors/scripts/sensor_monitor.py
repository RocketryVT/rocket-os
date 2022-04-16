#! /usr/bin/env python3

'''
Sensor Monitor: Monitors and Displays the data streams of various sensors.

			  + Num of Functions: 4
			  + Num of Classes: 1
'''

from time import time
import rospy
from std_msgs.msg import String, Bool, Float32
from sensors.msg import SensorReading
import csv
import numpy
import re

sensor_timer = None
voltage_timer = None

class Tracker:
	'''
		Tracks and displays a specific stream of sensor data.
	'''

    def __init__(self, name, unit=None):
        self.name = name
        self.last = None

    def get(self, message):
        '''
		    Update Tracker with current sensor data
	    '''

        self.last = message

    def to_string(self, voltage=False):
        '''
		    Put tracked sensor data in a string of the following format:
                ACRONYM: Data UnitType

            @param voltage: 
                - True - Convert raw Voltage Data to string
                - False - Convert SI Unit Data to string
    	'''

        # Create Acronym from Tracker Name
        acronym = ''.join([x[0] for x in self.name.split()]).upper()
        # Return Empty 
        if self.last is None:
            return "{}:     ".format(acronym)
        # Return only raw voltage data
        if voltage:
            return "{}: {:0.2f} V".format(acronym, self.last.voltage)
        # Return only data with no SI Unit value
        if self.last.unit == "":
            return "{}: {:0.2f}".format(acronym, self.last.reading)
        # Return only SI unit data 
        return "{}: {:0.2f} {}".format(acronym, self.last.reading, self.last.unit)

    def get_data(self):
        '''
            Returns the current data reading
        '''
        return self.last.reading


def echo_sensors(event=None):
	'''
		Displays the SI Unit data for all Trackers
	'''

    echo(False)


def echo_voltage(event=None):
	'''
		Displays the voltage data for all Trackers
	'''

    echo(True)


def echo(print_voltage):
	'''
		Prints the current data for every Tracker to screen,
        each separated by a "/".
        @param print_voltage: 
            - True - Prints Raw Voltage value
            - False - Prints SI Unit value
	'''

    output = ""
    for i, tracker in enumerate(trackers):
        output += tracker.to_string(print_voltage)
        if i < len(trackers) - 1:
            output = output + " / "
    rospy.loginfo(output) # Print


def get_command(message):
	'''
		Initiate commands pertaining to the display of sensor data.
            - read data
            - read data [0-9]+
            - stop data
            - read voltage
            - read voltage [0-9]+
            - stop voltage
	'''

    global sensor_timer
    global voltage_timer

    command = message.data

    # Start displaying sensor data
    if command == "read data":
        echo_sensors()

    elif bool(re.match(re.compile("^read data [0-9]+"), command)):

        period = float(command.split()[2]) # Num of Seconds to read data
        if sensor_timer:
            sensor_timer.shutdown()
        sensor_timer = rospy.Timer(rospy.Duration(period), echo_sensors)

    # Stop displaying sensor data
    elif command == "stop data":

        if sensor_timer:
            sensor_timer.shutdown()
        sensor_timer = None

    # Start displaying sensor voltage data
    elif command == "read voltage":
        echo_voltage()

    elif bool(re.match(re.compile("^read voltage [0-9]+"), command)):

        period = float(command.split()[2])
        if voltage_timer:
            voltage_timer.shutdown()
        voltage_timer = rospy.Timer(rospy.Duration(period), echo_voltage)

    # Stop displaying sensor voltage data
    elif command == "stop voltage":

        if voltage_timer:
            voltage_timer.shutdown()
        voltage_timer = None

def makeCSV():
    '''
        Build CSV file from sensor data
    '''
    csv_w.writerow([time_count, tracker[0].get_data, tracker[1].get_data,
        tracker[2].get_data, tracker[3].get_data, tracker[4].get_data])
    time_count += 1   



if __name__ == "__main__":

	# Set up CSV file
    csv_w = csv.writer(open("data.csv", 'w'))
    csv_w.writerow(["Time", "Pressure 1", "Pressure 2", "Temperature 1", "Temperature 2", "Temperature 3"])
    global time_count
    time_count = 0
    
    #Initialize Node
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

	#Set Subscriptions
    rospy.Subscriber("/sensors/ox_tank_transducer", SensorReading, trackers[0].get)
    rospy.Subscriber("/sensors/combustion_transducer", SensorReading, trackers[1].get)
    rospy.Subscriber("/sensors/ox_tank_thermocouple", SensorReading, trackers[2].get)
    rospy.Subscriber("/sensors/combustion_thermocouple_1", SensorReading, trackers[3].get)
    rospy.Subscriber("/sensors/combustion_thermocouple_2", SensorReading, trackers[4].get)
    rospy.Subscriber("/sensors/float_switch", SensorReading, trackers[5].get)

    rospy.spin()

    rospy.Timer(rospy.Duration(1), makeCSV)

