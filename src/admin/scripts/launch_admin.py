#! /usr/bin/env python

'''
Launch Admin: Manages the Launch Countdown and Launch of the vehicle.

			  Available Actions: 
				  + Start Countdown to Launch
				  + Ignite vehicle
				  + Abort Launch (Stop Countdown)
						
			  + Num of Functions: 5
'''

import rospy
from std_msgs.msg import String, UInt8, Empty

def decrement_countdown(event):

	'''
		Decrement the count down to Launch 
		Launch = T - 0
		
		@param event: Not used
	'''

    global remaining
    
    if remaining == 0:
        rospy.logwarn("T - {} seconds.".format(remaining))
        no_turning_back()
        return

    rospy.logwarn("T - {} seconds.".format(remaining))
    remaining -= 1


def no_turning_back():

	'''
		Ignites the vehicle for launch.
		Once this function is called, there is indeed no turning back.
	'''

    # execute this if we definitely want to launch
    stop_timer()
    # seriously, there's no way to stop the funk
    # if this function is called
    rospy.logwarn("Executing launch sequence.")
    rospy.sleep(0.5)

    abort_valve.publish(1) # close abort valve
    rospy.sleep(1)
    injection_valve.publish(2) # open injection valve
    rospy.sleep(0.4)
    ematch.publish() # fire ematches
    
    pub_commands.publish("system fortune | cowsay")


def start_timer():

	'''
		Starts the Launch countdown timer
	'''

    global remaining
    global countdown_timer
    remaining = countdown_maximum
	
	#Frequency that messages are published to the Topic
    countdown_timer = rospy.Timer(rospy.Duration(1), decrement_countdown)


def stop_timer():

	'''
		Stops the Launch countdown timer
	'''

    global remaining
    global countdown_timer
    remaining = countdown_maximum
    countdown_timer.shutdown()
    countdown_timer = None


def get_command(message):

	'''
		Begins the countdown timer if it has not started.
		
		If countdown timer is already running, 
		passing any command to this function will abort the launch
		
		@param message: The launch command [launch]
	'''

    command = message.data

    if command == "launch" and countdown_timer is None:
        rospy.logwarn("Beginning launch countdown. Any user input will abort the launch.")
        start_timer()

    elif countdown_timer is not None:
        stop_timer()
        rospy.logwarn("User input detected -- launch aborted.")


if __name__ == "__main__":

    countdown_maximum = 10 # number of seconds to count down from
    remaining = countdown_maximum
    countdown_timer = None

	#Initialize Node
    rospy.init_node("launch_admin")
	
	#Set Subscriptions
    rospy.Subscriber("/commands", String, get_command)
	
	#Set Publisher
    pub_commands = rospy.Publisher("/requested_commands", String, queue_size=10)

    # launch involves injection valve, ematch, abort valve
    injection_valve = rospy.Publisher("/hardware/injection_valve/request", UInt8, queue_size=10)
    abort_valve = rospy.Publisher("/hardware/abort_valve/request", UInt8, queue_size=10)
    ematch = rospy.Publisher("/hardware/ematch/command", Empty, queue_size=10)

    rospy.spin()

