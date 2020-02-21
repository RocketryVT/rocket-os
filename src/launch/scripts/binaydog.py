from datetime import datetime
import rospy
import sys
import random
from std_msgs.msg import String, Float32
from rosgraph_msgs.msg import Log



rospy.init_node("binay_dog", log_level=rospy.DEBUG)

pub_command = rospy.Publisher("/binaytime", Float32, queue_size=10)


start_time = datetime.now()


def keep_time():
    uptime = start_time - datetime.now()
    pub_command.publish(uptime)



def main():
    global start


print(start_time - datetime.now())
keep_time()


rospy.spin()







