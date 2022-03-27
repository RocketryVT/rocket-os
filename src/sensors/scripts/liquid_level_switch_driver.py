#! /usr/bin/env python3

# liquid_level_switch_driver.py

'''
Liquid Level Switch: Senses when liquid level has reached
                     a predetermined limit and returns a HIGH state.

                    + Num of Functions: 2
'''

try:
    import Adafruit_BBIO.ADC as adc
except:
    adc = None

import rospy
from sensors.msg import SensorReading
import sys


def voltage_to_state(voltage):
    '''
        Any voltage above 1.6V returns a HIGH state
        Any voltage 1.6V or below returns a LOW state

        @param voltage: The voltage input
        @return: The pin's state based on the voltage reading (HIGH or LOW)
    '''
    return voltage > 1.6


def read_and_publish(event):
    '''
        Read ADC values
        Convert ADC Values to Voltage 
        Determine State of Pin (Based off voltage info)
        Send Publish Voltage and Pin State data

        @param event: Not used
    '''

    global sequence_number

    if adc:
        # Convert ADC reading to Voltage
        voltage = adc.read(adc_pin)*1.8
    else:
        voltage = 0

    # Convert Voltage to HIGH or LOW State.
    state = voltage_to_state(voltage)

    msg = SensorReading()

    # Add relevant Data to message
    msg.header.seq = sequence_number
    sequence_number += 1
    msg.header.stamp = rospy.Time.now()
    msg.voltage = voltage
    msg.reading = state
    msg.unit = ""

    # Publish message
    publisher.publish(msg)


if __name__ == "__main__":

    sequence_number = 0

    # Initialize Node
    rospy.init_node("float_switch_driver", log_level=rospy.DEBUG)

    if adc:
        adc.setup()
    else:
        rospy.logwarn(
            "Failed to import Adafruit_BBIO.ADC, running in desktop mode")

    name = rospy.get_name()
    try:
        # Return Values From Parameter Server
        adc_pin = rospy.get_param(name + "/pin")  # - Get ADC Pin Num
        # - Get ADC message Transmission Frequency (sec)
        period = rospy.get_param(name + "/period")
    except:
        rospy.logerr("Failed to retrieve configuration from rosparam server.")
        rospy.signal_shutdown("Unavailable config.")
        exit()

    # Check if adc_pin matches AIN1 - AIN6. If not Exit program.
    all_pins = ["AIN{}".format(x) for x in range(0, 7)]
    if adc_pin not in all_pins:
        rospy.logerr("Provided pin " + adc_pin +
                     " not a valid ADC pin (" + str(all_pins) + ")")
        exit()

    # Set Publisher
    publisher = rospy.Publisher(name, SensorReading, queue_size=10)

    # Set message Publishing Frequency
    rospy.Timer(rospy.Duration(period), read_and_publish)

    rospy.loginfo(("Starting float switch driver on ADC {} " +
                   "and polling period {} seconds.").format(adc_pin, period))

    rospy.spin()
