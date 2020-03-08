#! /usr/bin/env python

# thermocouple_driver.py

import Adafruit_BBIO.ADC as adc
import rospy
from std_msgs.msg import Float32
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

    voltage = adc.read(adc_pin)*1.8
    celsius = voltage_to_celsius(voltage)
    pub_volt.publish(voltage)
    pub_temp.publish(celsius)


adc.setup()
rospy.init_node("thermocouple_driver");

sys.argv = rospy.myargv(sys.argv)
if len(sys.argv) < 2:
    rospy.logerr("Thermocouple driver requires ADC pin in args")
    exit()

adc_pin = sys.argv[1]
all_pins = ["AIN{}".format(x) for x in range(0,7)]
if adc_pin not in all_pins:
    rospy.logerr("Provided pin " + adc_pin + " not a valid ADC pin (" + str(all_pins) + ")")
    exit()

name = rospy.get_name()
pub_temp = rospy.Publisher(name + "/temperature", Float32, queue_size=10);
pub_volt = rospy.Publisher(name + "/voltage", Float32, queue_size=10);
rospy.Timer(rospy.Duration(0.2), read_and_publish)

rospy.loginfo("Starting thermocouple driver on ADC " + adc_pin)

rospy.spin()

