#! /usr/bin/env python3

from datetime import datetime
import rospy
import sys
import rosnode
from std_msgs.msg import String, Float32
from std_msgs.msg import Duration
import ros
from rosgraph_msgs.msg import Log


def publish_time(event):
    uptime = Duration()
    uptime.data = event.current_real - start_time
    pub_uptime.publish(uptime)


def print_time(event):
    uptime = Duration()
    uptime.data = event.current_real - start_time
    rospy.logdebug("Runtime has reached: " +
                   str(int(uptime.data.secs/60)) + " minutes.")


def check_on_nodes(event):
    global old_nodes
    new_nodes = set(rosnode.get_node_names())
    changed = new_nodes.symmetric_difference(old_nodes)

    for node in changed:
        if node in old_nodes:
            rospy.loginfo(node + " has died.")
        if node in new_nodes:
            rospy.loginfo(node + " has been born.")
    old_nodes = new_nodes


def get_command(message):

    cmd = message.data

    if cmd == "rosnode list":
        nodes = rosnode.get_node_names()
        nodes.sort()
        output = "\n" + "\n".join(nodes) + "\n"
        rospy.loginfo(output)

    if cmd == "rostopic list":
        topics = rospy.get_published_topics()
        topics.sort(key=lambda x: x[0])
        output = "\n" + \
            "\n".join([x.ljust(60) + y for (x, y) in topics]) + "\n"
        rospy.loginfo(output)

    if cmd == "uptime":
        seconds = (rospy.get_rostime() - start_time).secs
        hours = int(seconds/3600)
        seconds -= hours*3600
        minutes = int(seconds/60)
        seconds -= minutes*60
        rospy.loginfo(("Uptime has reached {} hours, {} minutes, " +
                       "{} seconds.").format(hours, minutes, seconds))


if __name__ == "__main__":

    rospy.init_node("watchdog", log_level=rospy.DEBUG)

    pub_uptime = rospy.Publisher("/uptime", Duration, queue_size=10)

    start_time = rospy.get_rostime()
    old_nodes = set()

    rospy.Timer(rospy.Duration(10), check_on_nodes)
    rospy.Timer(rospy.Duration(60), print_time)
    rospy.Timer(rospy.Duration(1), publish_time)
    rospy.Subscriber("/commands", String, get_command)

    rospy.spin()
