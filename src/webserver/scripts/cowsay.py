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

    if tokens[0]=="subtract":
        try:
            subtract=float(tokens[1])-float(tokens[2])
            rospy.loginfo("Difference is " +str(subtract))
        except:
            rospy.logerr("Error parsing command.")

    # exercise for the reader: implement subtract, multiply, and divide commands
    #
    # subtract 5 2 -> "difference is 3"
    # multiply 3 3 -> "product is 9"
    # divide 6 4 -> "quotient is 1.5"
    
def listener():

    rospy.init_node('cowsay')
    rospy.Subscriber("/webserver/commands", String, callback)

    rospy.spin()

if __name__ == '__main__':
    listener()
