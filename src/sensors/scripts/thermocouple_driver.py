#! /usr/bin/env python

# thermocouple_driver.py

import Adafruit_BBIO.ADC as adc
import rospy
from sensors.msg import SensorReading
import sys

def voltage_to_celsius(voltage):

    return (voltage**8)*-115.855752725240 + \
           (voltage**7)*999.436833871484 + \
           (voltage**6)*-3658.156796841630 + \
           (voltage**5)*7377.417876749070 + \
           (voltage**4)*-8879.836831 + \
           (voltage**3)*6439.970948 + \
           (voltage**2)*-2711.355273 + \
           (voltage)*1106.923311 - 198.2593002


def read_and_publish(event):

    global sequence_number

    voltage = adc.read(adc_pin)*1.8
    celsius = voltage_to_celsius(voltage)
    msg = SensorReading()

    msg.header.seq = sequence_number
    sequence_number += 1
    msg.header.stamp = rospy.Time.now()

    msg.voltage = voltage
    msg.reading = celsius
    msg.unit = "celsius"

    publisher.publish(msg)


if __name__ == "__main__":

    sequence_number = 0

    adc.setup()
    rospy.init_node("thermocouple_driver");

    adc_pin = sys.argv[1]
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

    rospy.loginfo(("Starting thermocouple driver on ADC {} " +
                   "and polling period {} seconds.").format(adc_pin, period))

    rospy.spin()

