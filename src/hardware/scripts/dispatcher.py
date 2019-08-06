#!/usr/bin/env python
import rospy
from std_msgs.msg import UInt8, String

motor_commands = {
    "stop": 0,
    "cw": 1,
    "ccw": 2
}

solenoid_commands = {
    "close": 0,
    "open": 1
}

topic_format = "/hardware/{}/command"
publishers = {}

def recieve_command(data):

    tokens = data.data.split(" ")

    if tokens[0] == "motor":
        try:
            command = motor_commands[tokens[1]]
            topic = topic_format.format(tokens[2])
            if topic not in publishers:
                publishers[topic] = rospy.Publisher(topic, UInt8, queue_size=10)
                rospy.sleep(0.5)
            publishers[topic].publish(command)
        except Exception as e:
            rospy.logerr("""Error parsing command - expected """
                """'motor {stop|cw|ccw} name'""")
            rospy.logerr("Exception: " + str(e))

    elif tokens[0] == "solenoid":
        try:
            command = solenoid_commands[tokens[1]]
            topic = topic_format.format(tokens[2])
            if topic not in publishers:
                publishers[topic] = rospy.Publisher(topic, UInt8, queue_size=10)
                rospy.sleep(0.5)
            publishers[topic].publish(command)
        except Exception as e:
            rospy.logerr("""Error parsing command - expected """
                """'solenoid {close|open} name'""")
            rospy.logerr("Exception: " + str(e))

rospy.init_node('dispatcher')
rospy.Subscriber("/webserver/commands", String, recieve_command)
rospy.spin()
