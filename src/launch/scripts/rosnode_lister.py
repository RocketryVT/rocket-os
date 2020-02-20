#! /usr/bin/env python

import rospy
from std_msgs.msg import String
import rosnode

def get_command(message):

    cmd = message.data

    if cmd == "rosnode list":
        output = "\n" + "\n".join(rosnode.get_node_names()) + "\n"
        rospy.loginfo(output)
    
    if cmd == "rostopic list":
        output = "\n" + "\n".join([x.ljust(60) + y for (x, y) in rospy.get_published_topics()]) + "\n"
        rospy.loginfo(output)

print(dir(rosnode.rosgraph.network))

rospy.init_node("rosnode_lister")
pub = rospy.Subscriber("/commands", String, get_command)
rospy.spin()
