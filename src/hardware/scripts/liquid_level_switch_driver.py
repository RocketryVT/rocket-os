#! /usr/bin/env python

# liquid_level_switch_driver.py

import rospy
from std_msgs.msg import Bool

def read_and_publish(event):

    # get liquid level switch measurement
    state = False
    publisher.publish(state)

rospy.init_node("liquid_level_switch_driver");
name = rospy.get_name()

publisher = rospy.Publisher(name + "/state", Bool, queue_size=10);

rospy.Timer(rospy.Duration(1), read_and_publish)

rospy.loginfo("Starting liquid level switch driver.")

rospy.spin()

