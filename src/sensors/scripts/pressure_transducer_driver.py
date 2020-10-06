#! /usr/bin/env python

# thermocouple_driver.py

try:
    import Adafruit_BBIO.ADC as adc
except:
    adc = None

import rospy
from sensors.msg import SensorReading
from std_msgs.msg import Float32, String
import sys
import json


def publish_vis(event):

    pub_vis.publish(json.dumps(dictionary))


def voltage_to_pressure(voltage):

    intercept = -106;
    slope = 50/0.08;
    return slope*voltage + intercept;


def read_and_publish(event):

    global sequence_number
    global dictionary

    if adc:
        voltage = adc.read(adc_pin)*1.8
    else:
        voltage = 0
    pressure = voltage_to_pressure(voltage)
    msg = SensorReading()

    dictionary = { "voltage": voltage, "pressure": pressure }

    msg.header.seq = sequence_number
    sequence_number += 1
    msg.header.stamp = rospy.Time.now()
    msg.voltage = voltage
    msg.reading = pressure
    msg.unit = "psig"

    publisher.publish(msg)


if __name__ == "__main__":

    sequence_number = 0

    rospy.init_node("pressure_transducer_driver", log_level=rospy.DEBUG)

    if adc:
        adc.setup()
    else:
        rospy.logwarn("Failed to import Adafruit_BBIO.ADC, running in desktop mode")

    name = rospy.get_name()
    try:
        adc_pin = rospy.get_param(name + "/pin")
        period = rospy.get_param(name + "/period")
        vis_period = rospy.get_param(name + "/vis_period")
    except:
        rospy.logerr("Failed to retrieve configuration from rosparam server.")
        rospy.signal_shutdown("Unavailable config.")
        exit()

    all_pins = ["AIN{}".format(x) for x in range(0,7)]
    if adc_pin not in all_pins:
        rospy.logerr("Provided pin " + adc_pin + \
            " not a valid ADC pin (" + str(all_pins) + ")")
        exit()

    publisher = rospy.Publisher(name, SensorReading, queue_size=10);
    pub_vis = rospy.Publisher("/vis_update", String, queue_size=10);
    rospy.Timer(rospy.Duration(period), read_and_publish)
    rospy.Timer(rospy.Duration(vis_period), publish_vis)
    
    rospy.loginfo(("Starting pressure transducer driver on ADC {} " +
                   "and polling period {} seconds.").format(adc_pin, period))
    rospy.spin()

