#! /usr/bin/env python

# pressure_transducer_driver.py

import rospy
from std_msgs.msg import Float32
from random import random 
import time

now = time.time()

def read_and_publish(event):

    # get pressure transducer measurement (psig)
    pressure = (time.time() - now)*3 # random()*1500
    publisher.publish(pressure)

rospy.init_node("pressure_transducer_driver");
name = rospy.get_name()

publisher = rospy.Publisher(name + "/pressure", Float32, queue_size=10);

rospy.Timer(rospy.Duration(0.1), read_and_publish)

rospy.loginfo("Starting pressure transducer driver.")

rospy.spin()

