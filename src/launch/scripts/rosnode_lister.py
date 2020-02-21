#! /usr/bin/env python

import rospy
from std_msgs.msg import String
import rosnode

def get_command(message):

    cmd = message.data

    if cmd == "rosnode list":
        nodes = rosnode.get_node_names()
        nodes.sort()
        output = "\n" + "\n".join(nodes) + "\n"
        rospy.loginfo(output)
    
    if cmd == "rostopic list":
        topics = rospy.get_published_topics()
        topics.sort(key=lambda x: x[0])
        output = "\n" + "\n".join([x.ljust(60) + y for (x, y) in topics]) + "\n"
        rospy.loginfo(output)

print(dir(rosnode.rosgraph.network))

rospy.init_node("rosnode_lister")
pub = rospy.Subscriber("/commands", String, get_command)
rospy.spin()
