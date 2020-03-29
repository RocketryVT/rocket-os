#! /usr/bin/env python

import rospy
from std_msgs.msg import Float32, UInt8
from sensors.msg import SensorReading


def receive_message(msg):

    global current_pressure 
    global active_control

    current_pressure = msg.reading

    if PRESSURE_TOO_HIGH <= current_pressure and not active_control:
        pressure_high()
        active_control = True
    elif PRESSURE_OK >= current_pressure and active_control:
        pressure_ok()
        active_control = False


def pressure_high():

    rospy.logwarn("Pressure is Too high")
    pub.publish(2)


def pressure_ok():

    rospy.loginfo("vent valve is not longer controled by pressure release admin")
    pub.publish(1)    


def publish_pressure(event):
    if active_control:
         rospy.loginfo("Control Pressure Status: " + str(active_control) + " and vibing, Pressure: "+ str(current_pressure)) 


if __name__ == "__main__":

    PRESSURE_TOO_HIGH = 1100 
    PRESSURE_OK = 900

    active_control = False
    current_pressure = None

    rospy.init_node("release_admin", log_level=rospy.DEBUG)

    sub = rospy.Subscriber("/sensors/ox_tank_transducer", SensorReading, receive_message)
    pub = rospy.Publisher("/hardware/vent_valve/request", UInt8, queue_size=10)

    check_timer = rospy.Timer(rospy.Duration(5), publish_pressure)

    rospy.spin()

