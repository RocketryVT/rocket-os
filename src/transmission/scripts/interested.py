#!/usr/bin/env python

# interested.py

import rospy
from transmission.msg import Packet

def recieve_packet(msg):

    global pub
    if msg.id % 28 == 0:
        rospy.loginfo("Got a message where id % 28 == 0: " + str(msg.time))
        pub.publish(msg);

def main():

    global pub
    rospy.init_node('interested')
    pub = rospy.Publisher('outgoing', Packet, queue_size=100)
    sub = rospy.Subscriber('incoming', Packet, recieve_packet)
    rospy.spin()

if __name__ == '__main__':
    main()
