
# this is not a node, and is not executable


'''
Driver Library: Brief Description

'''

import rospy
import yaml


VERBOSE = False

all_commands = {}


def cmd2str(msg):

	'''
		Convert the message details to a string
		
		@param msg: Messgae to convert to a string
	'''
	
    if msg is None:
        return str(msg)
    return "{}: {} ({:0.3f} s), P{}".format(
        msg.source,
        msg.command,
        msg.pulse.to_sec(),
        msg.priority)


def get_highest_priority_command():

	'''
		
	'''
	
    selected = None
    for cmd in all_commands.values():
        if VERBOSE:
            rospy.logdebug(">> " + cmd2str(cmd))
        if selected is None or cmd.priority > selected.priority:
            selected = cmd
        elif cmd.priority == selected.priority:
            if cmd.header.stamp < selected.header.stamp:
                selected = cmd
    if VERBOSE:
        rospy.logdebug("HPC: " + cmd2str(selected))
    return selected


def nullify_command(msg):

	'''
		
	'''
	
    if VERBOSE:
        rospy.logdebug("Nullifying command: " + cmd2str(msg))
    old_cmd = get_highest_priority_command()
    all_commands.pop(msg.source)
    new_cmd = get_highest_priority_command()
    if new_cmd == old_cmd:
        if VERBOSE:
            rospy.logdebug("HPC remains unchanged. Doing nothing.")
    elif new_cmd is not None:
        if VERBOSE:
            rospy.loginfo("Executing queued command from " + msg.source + ".")
        execute_command(new_cmd)


def receive_command(msg):

	'''
		
	'''
	
	# Notify that command was received
    if VERBOSE:
        rospy.loginfo("Received new command from " + msg.source + \
            ": " + str(msg.command))
    old_cmd = get_highest_priority_command()
    
	# Overwrite any previous command from this source
    all_commands[msg.source] = msg
    if msg.command == msg.RELEASE:
        rospy.loginfo(msg.source + " has released control of this driver.")
        all_commands.pop(msg.source)
    new_cmd = get_highest_priority_command()
	if new_cmd == old_cmd:
        rospy.loginfo("HPC remains unchanged. Doing nothing.")
	elif new_cmd is not None:
        if new_cmd.source != msg.source: 
            rospy.loginfo("Executing queued command from " + \
                new_cmd.source + ".")
        execute_command(new_cmd)


def callback(func):

	'''
		
	'''
    global execute_command
    execute_command = func


# do not change -- this is an abstract function, and will be
# overridden by particular driver node implementations
def execute_command(command):

	'''
		
	'''
	
    rospy.logfatal("Please overwrite this function!")
    rospy.signal_shutdown("Unimplemented driver callback")
    exit()


