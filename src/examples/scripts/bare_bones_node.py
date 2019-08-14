#!/usr/bin/env python

# bare_bones_node.py

import rospy

# initialize the node
rospy.init_node("bare_bones_python")

# do work here

# pass control back to ROS -
# this isn't strictly necessary with python nodes,
# but will prevent the process from exiting until we
# or ROS tell it to
rospy.spin()
