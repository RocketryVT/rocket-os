#!/usr/bin/env python

# vis_spewer.py

from thread import *
import sched
import random
import rospy
from std_msgs.msg import String, Float32

def publish_rand(event):
    value = random.randint(0, 10000)
    pub_random.publish("" + str(value) + "")


if __name__ == "__main__":

    #Init server node
    rospy.init_node("vis_spewer", log_level=rospy.DEBUG)
    name = rospy.get_name()

    pub_random = rospy.Publisher("/vis_update", String, queue_size=10)

    rospy.Timer(rospy.Duration(2), publish_rand)
    rospy.spin()

    

