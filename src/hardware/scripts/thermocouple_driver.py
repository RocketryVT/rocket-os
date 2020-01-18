#! /usr/bin/env python

# thermocouple_driver.py

import rospy
from std_msgs.msg import Float32

def read_and_publish(event):

    # get thermocouple measurement
    temp = 63.28
    publisher.publish(temp)

rospy.init_node("thermocouple_driver");
name = rospy.get_name()

publisher = rospy.Publisher(name + "/temperature", Float32, queue_size=10);

rospy.Timer(rospy.Duration(1), read_and_publish)

rospy.loginfo("Starting thermocouple driver.")

rospy.spin()

