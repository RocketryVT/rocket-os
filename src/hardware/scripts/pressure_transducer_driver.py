#! /usr/bin/env python

# pressure_transducer_driver.py

import rospy
from std_msgs.msg import Float32

def read_and_publish(event):

    # get pressure transducer measurement (psig)
    pressure = 701.4
    publisher.publish(pressure)

rospy.init_node("pressure_transducer_driver");
name = rospy.get_name()

publisher = rospy.Publisher(name + "/pressure", Float32, queue_size=10);

rospy.Timer(rospy.Duration(1), read_and_publish)

rospy.loginfo("Starting pressure transducer driver.")

rospy.spin()

