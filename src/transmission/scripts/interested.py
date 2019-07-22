#!/usr/bin/env python

# interested.py

import rospy
from transmission.msg import Packet

def recieve_packet(msg):
    if msg.id % 28 == 0:
        rospy.loginfo("Got a message where id % 28 == 0: " + str(msg.time))

def main():
    rospy.init_node('interested')
    rospy.Subscriber('incoming', Packet, recieve_packet)
    rospy.spin()

if __name__ == '__main__':
    main()
