#! /usr/bin/env python

import rospy
import rostopic
from std_msgs.msg import String
import re

def get_generic(msg):
    topic = msg._connection_header['topic']
    rospy.loginfo(topic + ": " + str(msg))

def listen_to(topic):

    if topic == "/rosout" or topic == "/rosout_agg":
        rospy.loginfo("Can't subscribe to /rosout or /rosout_agg.")
        return
    
    topics = [n for n, t in rospy.get_published_topics()]

    if topic not in topics:
        rospy.loginfo("Can't subscribe to " + topic + "; hasn't been published yet")
        return

    if topic in subscribed.keys():
        rospy.loginfo("Already subscribed to " + topic)
        return
        
    type = rostopic.get_topic_class(topic)
    if type:
        sub = rospy.Subscriber(topic, type[0], get_generic)
        subscribed[topic] = sub
        rospy.loginfo("Subscribed to " + topic)

def stop_listening(topic):

    if topic not in subscribed.keys():
        rospy.loginfo("Not currently listening to " + topic)
        return

    subscribed[topic].unregister()
    del subscribed[topic]
    rospy.loginfo("Stopped listening to " + topic)

def receive_command(msg):

    command = msg.data

    listen = "^listen to [\/\w.]*$"
    if bool(re.match(re.compile(listen), command)):
        listen_to(command.split()[2])

    stop = "^stop listening to [\/\w.]*$"
    if bool(re.match(re.compile(stop), command)):
        stop_listening(command.split()[3])


subscribed = {}

rospy.init_node("listener")
rospy.Subscriber("/commands", String, receive_command)
rospy.spin()
