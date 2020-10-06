#! /usr/bin/env python

import rospy
from std_msgs.msg import String
from hardware.msg import DriverCommand
import re
import yaml
import json


def on_shutdown():

    dc = DriverCommand()
    dc.header.stamp = rospy.Time.now()
    dc.header.seq = sequence_count
    dc.source = name
    dc.priority = 0
    dc.command = dc.RELEASE

    for publisher in publishers.values():
        publisher.publish(dc)

    rospy.loginfo("Exiting.")


def receive_command(msg):

    global sequence_count

    command = msg.data
    pattern = re.compile("[\\,\/,\w]+=[\\,\/,\w]+")

    if command == "driver help":

        rospy.loginfo("Submit a command to a command queue " + \
            "using the following syntax:\n\n\t" + \
            "driver topic=string priority=integer " + \
            "command=string pulse=seconds\n\nExample:\n\n\t" + \
            "driver topic=/hardware/solenoid priority=5 " +\
            "command=SOLENOID_ACTIVE pulse=2.3\n")

    elif command.startswith("driver"):
        dict = {}
        dc = DriverCommand()
        dc.header.seq = sequence_count
        sequence_count += 1
        dc.header.stamp = rospy.Time.now()
        topic = None
        dc.source = name
        for x in re.findall(pattern, command):
            key, value = x.split("=")
            dict[key] = value

            try:
                if key == "topic":
                    if value not in publishers.keys():
                        publishers[value] = rospy.Publisher(
                            value, DriverCommand, queue_size=10)
                        rospy.sleep(1)
                    topic = value
                elif key == "pulse":
                    dc.pulse = rospy.Duration(float(value))
                elif key == "priority":
                    dc.priority = int(value)
                elif key == "command":
                    if hasattr(dc, value.upper()):
                        dc.command = getattr(dc, value.upper())
                    else:
                        rospy.logerr("Invalid driver command: '" + value + \
                            "'.\nValid commands will be among these: " + \
                            ", ".join([x for x in dir(dc) if x.isupper()]))
                        return
            except Exception as e:
                rospy.logerr("Encountered an error parsing user command: " + \
                    str(type(e)) + ", " + str(e) + ".")

        if topic is None:
            rospy.logerr("No topic provided!")
            return

        rospy.logdebug(json.dumps(yaml.load(str(dc))))
        publishers[topic].publish(dc)


if __name__ == "__main__":

    publishers = {}
    sequence_count = 0

    rospy.init_node("driver_admin", log_level=rospy.DEBUG)
    name = rospy.get_name()
    rospy.Subscriber("/commands", String, receive_command)
    rospy.on_shutdown(on_shutdown)
    rospy.spin()

