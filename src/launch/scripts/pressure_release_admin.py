#! /usr/bin/env python

import rospy
from std_msgs.msg import Float32, UInt8
PRESSURE_TOO_HIGH = 1100 
PRESSURE_OK = 900
control = False
current_pressure = 0; 
def receive_message(msg):
    global current_pressure 
    current_pressure = msg.data
    global control
    if PRESSURE_TOO_HIGH <= msg.data and not control:
        pressure_high()	
        control = True
    elif PRESSURE_OK >= msg.data and control:
        pressure_ok()
	control = False

def pressure_high():
    rospy.logwarn("Pressure is Too high")
    pub.publish(2)

def pressure_ok():
    rospy.loginfo("vent valve is not longer controled by pressure release admin")
    pub.publish(1)    

def publish_pressure(event):
    if control:
         rospy.loginfo("Control Pressure Status: " + str(control) + " and vibing, Pressure: "+ str(current_pressure)) 
rospy.init_node("release_admin")
sub = rospy.Subscriber("/sensors/ox_tank_transducer/pressure", Float32, receive_message)
pub = rospy.Publisher("/hardware/vent_valve/command", UInt8, queue_size=10)
check_timer = rospy.Timer(rospy.Duration(5), publish_pressure)
rospy.spin()

