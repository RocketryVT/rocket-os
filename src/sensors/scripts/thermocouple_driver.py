#! /usr/bin/env python

# thermocouple_driver.py

'''
Thermocouple: A temperature Sensor.
			  Heating/Cooling of sensor creates a voltage
			  that can be correlated back to temperature.

			  + Num of Functions: 2
'''


try:
    import Adafruit_BBIO.ADC as adc
except:
    adc = None

import rospy
from sensors.msg import SensorReading
import sys

def voltage_to_celsius(voltage):

	'''
		Convert Voltage value to a Temperature (Celsius) value
		
		@param voltage: The voltage input
		@return: The converted temperature reading in Celsius
	'''
	
    return (voltage**8)*-115.855752725240 + \
           (voltage**7)*999.436833871484 + \
           (voltage**6)*-3658.156796841630 + \
           (voltage**5)*7377.417876749070 + \
           (voltage**4)*-8879.836831 + \
           (voltage**3)*6439.970948 + \
           (voltage**2)*-2711.355273 + \
           (voltage)*1106.923311 - 198.2593002


def read_and_publish(event):

	'''
		Read ADC values
		Convert ADC Values to Voltage 
		Determine Temperature in Celsius (Based off voltage info)
		Send Publish Voltage and Temperature (Celsius) data
		
		@param event: Not used
	'''

    global sequence_number

    if adc:
		#Convert ADC reading to Voltage
        voltage = adc.read(adc_pin)*1.8
    else:
        voltage = 0
		
	#Convert Voltage to Temperature (Celsius).
    celsius = voltage_to_celsius(voltage)
	
    msg = SensorReading()

	#Add relevant Data to message
    msg.header.seq = sequence_number
    sequence_number += 1
    msg.header.stamp = rospy.Time.now()
    msg.voltage = voltage
    msg.reading = celsius
    msg.unit = "celsius"

	#Publish message
    publisher.publish(msg)


if __name__ == "__main__":

    sequence_number = 0

	#Initialize Node
    rospy.init_node("thermocouple_driver", log_level=rospy.DEBUG);

    if adc:
        adc.setup()
    else:
        rospy.logwarn("Failed to import Adafruit_BBIO.ADC, running in desktop mode")

    adc_pin = sys.argv[1]
    name = rospy.get_name()
    try:
		#Return Values From Parameter Server 
        adc_pin = rospy.get_param(name + "/pin") # - Get ADC Pin Num
        period = rospy.get_param(name + "/period") # - Get ADC message Transmission Frequency (sec)
    except:
        rospy.logerr("Failed to retrieve configuration from rosparam server.")
        rospy.signal_shutdown("Unavailable config.")
        exit()

	#Check if adc_pin matches AIN1 - AIN6. If not Exit program.
    all_pins = ["AIN{}".format(x) for x in range(0,7)]
    if adc_pin not in all_pins:
        rospy.logerr("Provided pin " + adc_pin + " not a valid ADC pin (" + str(all_pins) + ")")
        exit()

	#Set Publisher. 
    publisher = rospy.Publisher(name, SensorReading, queue_size=10);
	
	#Set message Publishing Frequency
    rospy.Timer(rospy.Duration(period), read_and_publish)

    rospy.loginfo(("Starting thermocouple driver on ADC {} " +
                   "and polling period {} seconds.").format(adc_pin, period))

    rospy.spin()

