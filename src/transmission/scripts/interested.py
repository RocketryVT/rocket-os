#!/usr/bin/env python

# interested.py

import rospy
from transmission.msg import Packet

def recieve_packet(msg):
    rospy.loginfo("Got a message with timestamp " + str(msg.time))

def main():
    rospy.init_node('interested')
    rospy.Subscriber('/transmission/incoming',
        Packet, recieve_packet)
    rospy.spin()

if __name__ == '__main__':
    main()
