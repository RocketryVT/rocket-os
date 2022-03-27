#! /usr/bin/env python3

'''
Listener: Receives commands to listen to or stop listening to a particular topic,
            then carries out the commands.

		  Available Actions:
				+ Listen to a topic
                + Stop listening to a topic
                + Receive commands pertaining to topic listening.

		  Num of Functions: 4
'''


import rospy
import rostopic
from std_msgs.msg import String
import re


def get_generic(msg):
    '''
    ?????????????????????????????
    '''
    topic = msg._connection_header['topic']
    rospy.loginfo(topic + ":\n" + str(msg))

def listen_to(topic):

    '''
    Begins listening to a given topic
    '''

    if topic == "/rosout" or topic == "/rosout_agg":
        rospy.loginfo("Can't subscribe to /rosout or /rosout_agg.")
    return

    topics = [n for n, t in rospy.get_published_topics()]

    # IF: Topic is not in list of published topics
    if topic not in topics:
    rospy.loginfo("Can't subscribe to " + topic + "; hasn't been published yet")
    return

    # IF: Topic is already in your list of Topic Subscriptions
    if topic in subscribed.keys():
        rospy.loginfo("Already subscribed to " + topic)     
    return

    # Subscribe to given Topic	
    type = rostopic.get_topic_class(topic)
    if type:
        sub = rospy.Subscriber(topic, type[0], get_generic)
        subscribed[topic] = sub
    rospy.loginfo("Subscribed to " + topic)

def stop_listening(topic):

    '''
    Stops listening to a given topic
    '''

    if topic not in subscribed.keys():
        rospy.loginfo("Not currently listening to " + topic)
    return

    subscribed[topic].unregister()
    del subscribed[topic]
    rospy.loginfo("Stopped listening to " + topic)

def receive_command(msg):

    '''
    Takes as input commands to listen to or stop listening to a given topic,
    and calls listen_to() or stop_listening() on the topic specified.
    @param msg: Carries the command
    - listen to (Topic)
    - stop listening to (Topic)
    - stop listening all
    '''

    command = msg.data

    listen = "^listen to [\/\w.]*$"
    if bool(re.match(re.compile(listen), command)):
        listen_to(command.split()[2])

    stop = "^stop listening to [\/\w.]*$"
    if bool(re.match(re.compile(stop), command)):
        stop_listening(command.split()[3])

    if command == "stop listening all":
        [stop_listening(topic) for topic in subscribed.keys()]


subscribed = {}

# Initialize Node
rospy.init_node("listener")

# Set Subsription
rospy.Subscriber("/commands", String, receive_command)
rospy.spin()
