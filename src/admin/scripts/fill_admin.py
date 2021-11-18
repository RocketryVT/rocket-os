#! /usr/bin/env python


'''
Fill Admin: Manages the vehicle filling process

			Available Actions: 
				+ Start & Stop vehicle fill process
				+ Set state of Solenoid valve
				
			Data Managed:
				+ Pressure Data
				+ Liquid Level Data
				+ Loss of Signal Data
				+ Readiness Level

			Num of Functions: 9
'''

import rospy
from std_msgs.msg import String, Float32, UInt8
from sensors.msg import SensorReading
from hardware.msg import DriverCommand


def on_shutdown():
	
	'''
		Closes Solenoid Valve.
	'''
	
	#Add relevant Data to your message
    dc = DriverCommand()
    dc.header.stamp = rospy.Time.now()
    dc.source = name
    dc.priority = admin_priority
    dc.command = dc.RELEASE
    solenoid_cmd.publish(dc)

    rospy.loginfo("Exiting.")


def toggle_fill(state):
	
	'''
		Sets the state of the fill process according to the parameter passed.
		(HIGH = Fill / LOW = No Fill)
		
		@param state: True or False
	'''
	
    global fill_ongoing
    fill_ongoing = state
    set_solenoid(state)


def print_relevant_data(event):
	
	'''
		Prints & Logs Current Pressure and state of the LLS (Liquid Level Switch)
		
		@param event: Not used
	'''
	
    if fill_ongoing:
        rospy.loginfo("Current pressure: {} psig; LLS: {}".format(last_pressure, last_lls))


def get_pressure(message):
	
	'''
		Gets the current pressure.
		If the pressure is too high during an ongoing fill, 
		the fill is halted.
		
		@param message: SensorReading message type - pressure readings
	'''
	
    global last_pressure
    global fill_ongoing
    last_pressure = message.reading

    if message.reading > max_safe_pressure and fill_ongoing:
        rospy.logwarn("Maximum safe pressure exceeded -- stopping fill!")
        toggle_fill(False)


def get_lls_reading(message):
	
	'''
		Checks if the liquid level has reached its limit.
		If the Liquid Level Switch has been tripped (indicated by a True state)
		during an ongoing fill, the fill is halted.
		
		@param message: SensorReading message type - state of Liquid Level Switch
	'''
	
    global last_lls
    global fill_ongoing
    last_lls = message.reading

    if message.reading and fill_ongoing:
        rospy.logwarn("Liquid level switch tripped -- stopping fill!")
        toggle_fill(False)


def get_los(message):
	
	'''
		Checks for Loss of Signal.
		If Loss of Signal has been detected (indicated by a True state)
		during an ongoing fill, the fill is halted.
		
		@param message: Loss of Signal
	'''
	
    seconds = message.data
    if seconds and fill_ongoing:
        rospy.logwarn("LOS detected -- stopping fill!")
        toggle_fill(False)


def get_readiness(message):
	
	'''
		Get vehicle's current Level of Readiness
		
		@param message: A number indicating the Level of Readiness
	'''
	
    global current_readiness
    current_readiness = message.data


def set_solenoid(state):
	
	'''
		Sets Solenoid to Active or Inactive based on the given parameter
		
		@param state: True (Set to Active) or False (Set to Inactive)
	'''
	
	#Add relevant Data to your message
    dc = DriverCommand()
    dc.header.stamp = rospy.Time.now()
    dc.source = name
    dc.priority = admin_priority
    if state:
        dc.command = dc.SOLENOID_ACTIVE
    else:
        dc.command = dc.SOLENOID_INACTIVE
    solenoid_cmd.publish(dc)


def get_command(message):
	
	'''
		Starts or Stops the vehicle fill process based on the given parameter.
		
		@param message: fill commands [begin fill, end fill]
	'''
	
    global fill_ongoing
    command = message.data

    if command == "begin fill":
        if current_readiness is None:
            rospy.logwarn("Haven't received readiness update -- " + \
                "unsafe to continue.")
            return
        elif current_readiness != 2:
            rospy.logwarn("Fill only enabled for readiness level 2.")
            return
        rospy.logwarn("Beginning nitrous oxide fill.")
        toggle_fill(True)       
 
    elif command == "end fill":
        rospy.logwarn("Ending nitrous oxide fill.")
        toggle_fill(False)


if __name__ == "__main__":
	
    last_lls = None
    last_pressure = None
    fill_ongoing = False
    max_safe_pressure = 900
    admin_priority = 4
    current_readiness = None

	#Initialize Node
    rospy.init_node("fill_admin", log_level=rospy.DEBUG)
    name = rospy.get_name()

	#Set Subscriptions
    rospy.Subscriber("/commands", String, get_command)
    rospy.Subscriber("/los", Float32, get_los)
    rospy.Subscriber("/readiness_level", UInt8, get_readiness)
    rospy.Subscriber("/sensors/float_switch", SensorReading, get_lls_reading)
    rospy.Subscriber("/sensors/ox_tank_transducer", SensorReading, get_pressure)
	
	#Set Publisher
    solenoid_cmd = rospy.Publisher("/hardware/solenoid", DriverCommand, queue_size=10)

    rospy.sleep(1)

    rospy.loginfo("Assuming control of the solenoid.")
    set_solenoid(False)

    rospy.on_shutdown(on_shutdown)

	#Frequency that messages are published to the Topic
    rospy.Timer(rospy.Duration(3), print_relevant_data)

    rospy.spin()

