#! /usr/bin/env python

# liquid_level_switch_driver.py

import Adafruit_BBIO.ADC as adc
import rospy
from std_msgs.msg import Bool, Float32
import sys

def voltage_to_state(voltage):

    return voltage > 1.6

def read_and_publish(event):

    voltage = adc.read(adc_pin)*1.8
    state = voltage_to_state(voltage)
    pub_volt.publish(voltage)
    pub_state.publish(state)


adc.setup()
rospy.init_node("float_switch_driver");

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

pub_state = rospy.Publisher(name + "/state", Bool, queue_size=10);
pub_volt = rospy.Publisher(name + "/voltage", Float32, queue_size=10);
rospy.Timer(rospy.Duration(period), read_and_publish)

rospy.loginfo(("Starting float switch driver on ADC {} " +
               "and polling period {} seconds.").format(adc_pin, period))

rospy.spin()

