#! /usr/bin/env python

import rospy
from std_msgs.msg import String
from hardware.msg import DriverCommand
import signal

def signal_handler(frame, sig):
    print("")
    exit()

signal.signal(signal.SIGINT, signal_handler)

rospy.init_node("driver_commander", log_level=rospy.DEBUG)

while True:

    dc = DriverCommand()
    print("Topic:"),
    topic = raw_input()
    pub = rospy.Publisher(topic, DriverCommand, queue_size=10)
    print("Source:"),
    dc.source = raw_input()
    print("Command (release-0 stop-1 close-2 open-3 pulsec-4\npulseo-5 inactive-1 active-2 lock-1 fire-2):"),
    dc.command = int(raw_input())
    print("Pulse (sec):"),
    dc.pulse = rospy.Duration(float(raw_input()))
    print("Priority:"),
    dc.priority = int(raw_input())
    
    dc.header.stamp = rospy.Time.now()
    pub.publish(dc)

# uint8 RELEASE=0
# uint8 STOP=1
# uint8 CLOSE=2
# uint8 OPEN=3
# uint8 PULSE_CLOSE=4
# uint8 PULSE_OPEN=5
# uint8 INACTIVE=1
# uint8 ACTIVE=2
# uint8 LOCK=1
# uint8 FIRE=2
# std_msgs/Header header
#   uint32 seq
#   time stamp
#   string frame_id
# string source
# uint8 command
# uint8 priority
# duration pulse

