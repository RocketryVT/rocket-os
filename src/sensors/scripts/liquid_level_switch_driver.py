#! /usr/bin/env python

# liquid_level_switch_driver.py

import Adafruit_BBIO.ADC as adc
import rospy
from sensors.msg import SensorReading
import sys


def voltage_to_state(voltage):

    return voltage > 1.6


def read_and_publish(event):

    global sequence_number

    voltage = adc.read(adc_pin)*1.8
    state = voltage_to_state(voltage)
    msg = SensorReading()

    msg.header.seq = sequence_number
    sequence_number += 1
    msg.header.stamp = rospy.Time.now()

    msg.voltage = voltage
    msg.reading = state
    msg.unit = ""

    publisher.publish(msg)


if __name__ == "__main__":

    sequence_number = 0

    adc.setup()
    rospy.init_node("float_switch_driver", log_level=rospy.DEBUG);

    name = rospy.get_name()
    try:
        adc_pin = rospy.get_param(name + "/pin")
        period = rospy.get_param(name + "/period")
    except:
        rospy.logerr("Failed to retrieve configuration from rosparam server.")
        rospy.signal_shutdown("Unavailable config.")
        exit()

    all_pins = ["AIN{}".format(x) for x in range(0,7)]
    if adc_pin not in all_pins:
        rospy.logerr("Provided pin " + adc_pin + " not a valid ADC pin (" + str(all_pins) + ")")
        exit()

    publisher = rospy.Publisher(name, SensorReading, queue_size=10);
    rospy.Timer(rospy.Duration(period), read_and_publish)

    rospy.loginfo(("Starting float switch driver on ADC {} " +
                   "and polling period {} seconds.").format(adc_pin, period))

    rospy.spin()

