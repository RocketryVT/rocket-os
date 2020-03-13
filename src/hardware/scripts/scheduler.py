#! /usr/bin/env python

import rospy
from std_msgs.msg import String, Empty, UInt8, Bool


def get_command(message):

    command = message.data

    if command == "print reservation table":
        print_reservation_table()


def get_priority(node_name):
    if node_name not in priorities:
        return -1;
    return priorities.index(node_name)


def get_highest_priority(table):

    max_priority = -2
    max_node, max_command = None, None
    for node, command in table.items():
        prio = get_priority(node)
        if prio > max_priority:
            max_priority = prio
            max_node = node
            max_command = command
    return max_node, max_command


def receive_hardware_command(message):

    node = message._connection_header['callerid']
    topic = message._connection_header['topic']
    data = message.data

    (old_node, old_command) = get_highest_priority(reservation_tables[topic])

    if data == 255 and node in reservation_tables[topic].keys():
        del reservation_tables[topic][node]
        rospy.logdebug("Reservation for {} released by {}".format(topic, node))
    elif data != 255:
        reservation_tables[topic][node] = data

    (new_node, new_command) = get_highest_priority(reservation_tables[topic])

    if old_node == new_node and old_command == new_command:
        rospy.logdebug("Recieved. Reservation tables unchanged.")
    else:
        rospy.logdebug("Reservation for {} changed from {} ({}), {} to {} ({}), {}."
            .format(topic, old_node, get_priority(old_node), old_command,
                           new_node, get_priority(new_node), new_command))

    if new_node is not None:
        publishers[topic].publish(new_command)


def print_reservation_table():

    output = "\n\n"
    for topic, table in reservation_tables.items():
        output += "    " + topic + "\n"
        for node, value in table.items():
            output += "      {} ({}): {}\n".format(
                node, get_priority(node), value)
    rospy.loginfo(output)


if __name__ == "__main__":

    rospy.init_node("scheduler", log_level=rospy.DEBUG)

    rospy.Subscriber("/commands", String, get_command)

    priorities = [
        "/admin/fill",
        "/admin/pressure_release",
        "/hardware/dispatcher",
    ]

    hardware_topics = [
        ("injection_valve", UInt8),
        ("abort_valve", UInt8),
        ("solenoid", UInt8),
        ("linear_actuator", UInt8)]

    reservation_tables = {};
    publishers = {}

    for topic, type in hardware_topics:
        t = "/hardware/{}/request".format(topic)
        s = "/hardware/{}/command".format(topic)
        reservation_tables[t] = {}
        publishers[t] = rospy.Publisher(s, type, queue_size=10)
        rospy.Subscriber(t, type, receive_hardware_command)

    rospy.spin()

