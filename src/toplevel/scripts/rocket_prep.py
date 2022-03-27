#!/usr/bin/env python3
'''
Description: 9 Levels of Readiness

	- Very much unfinished
'''
import rospy
import time
from std_msgs.msg import String, UInt8, Bool, Float32
from hardware.msg import DriverCommand

try:
    import Adafruit_BBIO.GPIO as gpio
except:
    gpio = None


def set_sensorpins_low():
	'''
		Set all sensor pins low.
		@return:
			1 - success
			0 - error
	'''
	try:
		comb_thermo_1 = rospy.get_param("/sensors/combustion_thermocouple_1/pin")
		comb_thermo_2 = rospy.get_param("/sensors/combustion_thermocouple_2/pin")
		comb_transd = rospy.get_param("/sensors/combustion_transducer/pin")
		ox_thermo = rospy.get_param("/sensors/ox_tank_thermocouple/pin")
		ox_transd = rospy.get_param("/sensors/ox_tank_transducer/pin")
		float_sw = rospy.get_param("/sensors/float_switch/pin")

		pins = [comb_thermo_1, comb_thermo_1,
		    comb_transd, ox_thermo, ox_transd, float_sw]

	except:
		rospy.logerr("Failed to retrieve pin config from rosparam server.")
		return 0

	for pin in pins:
		try:
			if gpio:
				gpio.output(pin, gpio.LOW)
		except:
			rospy.logerr("Failed to set {} LOW.".format(pin))
			return 0

	return 1


def close_valves():
	'''
		Close all valves.
	'''

	abort_valve.publish(1)  # close abort valve
	injection_valve.publish(1)  # close injection valve

	# Solenoid command setup (Close solenoid)
    dc = DriverCommand()
    dc.header.stamp = rospy.Time.now()
    dc.source = name
    dc.priority = admin_priority
    dc.command = dc.SOLENOID_INACTIVE
    solenoid_cmd.publish(dc)




def update_readiness(current_level):
	'''
		Update the Readiness Level
	'''
	global readiness_level
	readiness_level = current_level.data



def update_idiot_status(idiot):
	'''
		Update the Idiot Check status
	'''
	global idiot_status
	idiot_status = idiot.data



def update_tankfull_state(full_state):
	'''
		Update state of tank liquid level
	'''
	global tank_full
	tank_full = full_state.data



def update_los(los):
	'''
		Update LOS Status
	'''
	global los_status
	los_status = los.data



def timer(secs, interrupt_enabled):
	'''
		Counts for a specified number of seconds.
		@oaram secs: Specified num of seconds.
		@oaram interrupt_enabled: If True, timer is stopped if signal is found.
									If False, timer is just a normal timer.
		@return:
			0 - Signal found.
			1 - "secs" seconds passed with no signal found.
	'''
	global los_status
	while secs:
		time.sleep(1)
		secs -= 1
		if interrupt_enabled:
			update_los()
			if los_status == 0.0:
				return 0
	return 1



def abort():
	'''
		Initiate abort sequence.
	'''

	# 1. Close Ignition Valve <-- The Hell is this?


	# 2. Detach the Umbilical <-- There has to be linear actuator node somewhere...


	# 3. Fully Open Vent & Abort Valve (Indefinitely)
	abort_valve.publish(2) # Open abort valve
	# what is the "vent"?

	rospy.logerr("20 min LOS. Aborting.")
	exit()





def level_master():
	'''
		This function might end up not being necessary...
		Calls readiness level procedure according to current readiness level.
		@param los_status: Amount of time signal has been lost. (0.0 sec if No LOS)
	'''
	global readiness_level
	global los_status

	if readiness_level == 0:
		start_level_0()
	if readiness_level == 1:
		start_level_1(los_status)
	if readiness_level == 2:
		start_level_2(los_status)
	if readiness_level == 3:
		start_level_3(los_status)
	if readiness_level == 4:
		start_level_4(los_status)
	if readiness_level == 5:
		start_level_5(los_status)
	if readiness_level == 6:
		start_level_6(los_status)
	if readiness_level == 7:
		start_level_7(los_status)
	if readiness_level == 8:
		start_level_8()
	if readiness_level == 9:
		start_level_9()



def start_level_0():
	'''
		- Set all pins LOW (Sensors and Continuity only)
		- Power Up Controller (WHERE IS THIS DONE???????)
		- Unlock all Hardware Interrupts (WHERE IS THIS DONE???????)
		- Connect to Controller via laptop
		Commands: read data | idot check
	'''

	global idiot_status
	# What is Continuity??
	if set_sensorpins_low() == 0:
		return

	if idiot_status:
		commands.publish("elevate readiness")

	else:
		return
		
		


def start_level_1(los_status):	
	'''
		Close all valves.
		Commands: read data | close valves | abort
		@param: los_status:
	'''

	close_valves()
		
	if los_status > 0.0:
		commands.publish("reduce readiness")
		return

	commands.publish("elevate readiness")





def start_level_2(los_status):
	'''
		Begin Fill process.
		Commands: begin fill | end fill | read data | open vent | crack vent | 
					close vent | abort
		@param: los_status:
	'''
	global tank_full
	fillCommands.publish("begin fill")
	if los_status > 0.0:
		# Elevate sensor monitor permissions to correct off-nominal readings

		if timer(1200, True) == 1:
			abort()
			
	else:
		if tank_full:
			commands.publish("elevate readiness")
	
		

		


def start_level_3(los_status):
	'''
		Reasses Rocket State Post-Fill (AND) Disconnect Umbilical
		Commands: end fill | read data | open vent | crack vent | close vent | 
					fill disconnect | abort |
					take me backwards daddy my name is dan and i am an 
					admin take me home country road	
		@param: los_status:
	'''	

	# Reasses rocket state
	
	# Disconnect umbillical (Is there a linear actuator node anywhere?).
	linact_a = rospy.get_param("/hardware/linear_actuator/pin_a")
	linact_b = rospy.get_param("/hardware/linear_actuator/pin_b")
	try:
			if gpio:
				# I don't know the HIGH-LOW sequence for this.
				gpio.output(linact_a, gpio.HIGH)
				gpio.output(linact_b, gpio.HIGH)
		except: 
			rospy.logerr("Failed to disconnect umbillical LOW.")
			return

	if los_status > 0.0:
		# Elevate sensor monitor permissions to correct off-nominal readings

		if timer(1200, True) == 1:
			abort()
			
	else:

		# Check for fill disconnect <-Whaaaa?
		
		commands.publish("elevate readiness")

		



def start_level_4(los_status):
	'''
		Enter a Holding Pattern before launch.
		Commands: open vent | crack vent | close vent | abort | read data | 
					idiot check part two point oh
		@param: los_status:
	'''

	global idiot_status
	if los_status > 0.0:
		# Elevate sensor monitor permissions to correct off-nominal readings

		if timer(1200, True) == 1:
			abort()
			
	else:
		if idiot_status:
			commands.publish("elevate readiness")
	
		
	


def start_level_5(los_status):
	'''
		FIRST LAUNCH READY PHASE: Close Vent & Abort Valve (With arm rocket command) 
		Commands: arm rocket | abort | crack vent | close vent | open vent
		@param: los_status:
	'''

	# Arm Rocket
	# What Vent to close?
	abort_valve.publish(1) # close abort valve

	if los_status > 0.0:
		# Elevate sensor monitor permissions to correct off-nominal readings

		if timer(1200, True) == 1:
			abort()
			
	else:
		# Check that rocket is armed. If true, publish.
		commands.publish("elevate readiness")

		



def start_level_6(los_status):
	'''
		SECOND LAUNCH READY PHASE: (get ready to rumble command) 
		Commands: get ready to rumble | read data | abort | 
					roll back to Level of Readiness five		
		@param: los_status:
	'''

	if los_status > 0.0:
		# Elevate sensor monitor permissions to correct off-nominal readings

		if timer(1200, True) == 1:
			abort()
			
	else:
		# Where to send "get ready to rumble" command?

		commands.publish("elevate readiness")

		


			
def start_level_7(los_status):
	'''
		THIRD LAUNCH READY PHASE: Launch is Now Allowed 
		Commands: launch the rocket | read data | abort | 
					roll back to Level of Readiness five	
		@param: los_status:
	'''

	if los_status > 0.0:
		# Elevate sensor monitor permissions to correct off-nominal readings

		if timer(1200, True) == 1:
			abort()
			
	else:
		commands.publish("launch")
		commands.publish("elevate readiness")





def start_level_8():
	'''
		The Countdown
		Commands: (anything) | abort
	'''

	# On completion of countdown
	# What are the commands "system fortune" and "cowsay"?
	commands.publish("elevate readiness")

		



def start_level_9():
	'''
		Post Launch
			+ Motor Fires
			+ Wait [20 min]
			+ Abort
	'''
	# Check that ematch has been lit I assume?
	timer(1200, False)
	abort()




if __name__ == "__main__":
    
	readiness_level = 0
	idiot_status = False
	tank_full = False
	los_status = 0.0
	admin_priority = 4 # Check for correct priority (Example in fill_admin)

	try:

		# Set Subscriptions
		rospy.Subscriber("/readiness_level", UInt8, update_readiness) 
		rospy.Subscriber("idiot_checker_one/status", Bool, update_idiot_status)
		rospy.Subscriber("tank_state", Bool, update_tankfull_state)
		rospy.Subscriber("/los", Float32, update_los)
		
		# Set Publishers
		commands = rospy.Publisher("/requested_commands", String, queue_size=10)
		fillCommands = rospy.Publisher("/commands", String, queue_size=10)
		injection_valve = rospy.Publisher("/hardware/injection_valve/request", UInt8, queue_size=10)
    	abort_valve = rospy.Publisher("/hardware/abort_valve/request", UInt8, queue_size=10)
		solenoid_cmd = rospy.Publisher("/hardware/solenoid", DriverCommand, queue_size=10)

		rospy.Timer(rospy.Duration(1), level_master)
		rospy.spin()


    except rospy.ROSInterruptException:
        pass	








