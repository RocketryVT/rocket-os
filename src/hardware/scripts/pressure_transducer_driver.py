#! /usr/bin/env python

# thermocouple_driver.py

import Adafruit_BBIO.ADC as adc
import rospy
from std_msgs.msg import Float32
import sys

def voltage_to_pressure(voltage):

    return voltage*900

def read_and_publish(event):

    voltage = adc.read(adc_pin)*1.8
    pressure = voltage_to_pressure(voltage)
    pub_volt.publish(voltage)
    pub_press.publish(pressure)


adc.setup()
rospy.init_node("pressure_transducer_driver")

sys.argv = rospy.myargv(sys.argv)
if len(sys.argv) < 2:
    rospy.logerr("Pressure transducer driver requires ADC pin in args")
    exit()

adc_pin = sys.argv[1]
all_pins = ["AIN{}".format(x) for x in range(0,7)]
if adc_pin not in all_pins:
    rospy.logerr("Provided pin " + adc_pin + " not a valid ADC pin (" + str(all_pins) + ")")
    exit()

name = rospy.get_name()
pub_press = rospy.Publisher(name + "/pressure", Float32, queue_size=10);
pub_volt = rospy.Publisher(name + "/voltage", Float32, queue_size=10);
rospy.Timer(rospy.Duration(1), read_and_publish)

rospy.loginfo("Starting pressure transducer driver on ADC " + adc_pin)

rospy.spin()

