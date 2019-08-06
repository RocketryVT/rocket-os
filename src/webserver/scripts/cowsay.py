#!/usr/bin/env python
import rospy
from std_msgs.msg import String

def callback(data):

    cmd = data.data
    tokens = cmd.split(" ")

    if tokens[0] == "add":
        try:
            sum = float(tokens[1]) + float(tokens[2])
            rospy.loginfo("Sum is: " + str(sum))
        except:
            rospy.logerr("Error parsing command.")
    
def listener():

    rospy.init_node('cowsay')
    rospy.Subscriber("/webserver/commands", String, callback)

    rospy.spin()

if __name__ == '__main__':
    listener()
