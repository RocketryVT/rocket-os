#! /usr/bin/env python

# thermocouple_driver.py

import rospy
from std_msgs.msg import Float32
from random import random
import time

now = time.time()

def read_and_publish(event):

    # get thermocouple measurement
    temp = (time.time() - now)*2 # random()*120
    publisher.publish(temp)

rospy.init_node("thermocouple_driver");
name = rospy.get_name()

publisher = rospy.Publisher(name + "/temperature", Float32, queue_size=10);

rospy.Timer(rospy.Duration(0.1), read_and_publish)

rospy.loginfo("Starting thermocouple driver.")

rospy.spin()

