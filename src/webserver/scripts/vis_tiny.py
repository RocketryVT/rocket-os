#! /usr/bin/env python

import rospy
from rosgraph_msgs.msg import Log
import yaml
import json

def callback(msg):
    print("{" + msg._connection_header['topic'] + ": " + json.dumps(yaml.load(str(msg))) + "}")


if __name__ == "__main__":

    rospy.init_node("vis_tiny", log_level=rospy.DEBUG)
    rospy.Subscriber("/rosout", Log, callback)
    rospy.spin()

