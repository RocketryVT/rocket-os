#!/usr/bin/env python

# interested.py

import rospy
from transmission.msg import Packet

def recieve_packet(data):
    rospy.loginfo(data.id)

def main():
    rospy.init_node('interested')
    rospy.Subscriber('recieved_packets',
        Packet, recieve_packet)
    rospy.spin()

if __name__ == '__main__':
    main()
